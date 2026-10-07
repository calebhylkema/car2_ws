// brake.cpp — car2 drive-by-wire BRAKE node (Teensy 4.1)
//
// Receives brake commands on CAN 0x300, drives a linear actuator through an
// H-bridge (RPWM/LPWM) with analog position feedback, and reports on CAN 0x301.
//
// Design highlights (see DESIGN.md §4):
//   - proportional closed-loop position control with a deadband (not bang-bang);
//     large errors still command full PWM, so full-brake application stays fast
//   - feedback is filtered (EMA) for the control loop, not just for status
//   - fail-safe (estop / stale command / rejected frame) = FULL brake applied
//   - command frames validated by the shared E2E layer (dbw_can.h)
//
// Measured installed hardware: pin 19 retracts/releases, pin 18 extends/applies,
// and position feedback on A0 decreases as the actuator extends. The measured
// endpoints are 945 released and 745 at the installed mechanical apply limit;
// normal control stops at 755 to retain a small apply-side safety margin.

#include <Arduino.h>
#include "dbw_can.h"

// ----------------------------- Configuration -----------------------------
namespace cfg {
    constexpr uint8_t  PIN_RETRACT = 19; // retract actuator, release brake
    constexpr uint8_t  PIN_EXTEND  = 18; // extend actuator, apply brake
    constexpr uint8_t  PIN_FBK     = A0; // potentiometer wiper

    constexpr int      ADC_RELEASED = 945;   // measured full retract / brake off
    constexpr int      ADC_APPLIED_LIMIT = 745; // installed mechanical maximum
    constexpr int      ADC_APPLIED  = 755;   // commanded maximum with 10-count margin
    constexpr float    STROKE_IN     = 2.0f; // for status reporting only

    constexpr int      DEADBAND_ADC  = 4;    // stop band around target
    constexpr float    KP            = 6.0f; // PWM per ADC count of error
    constexpr uint8_t  PWM_MIN       = 70;   // break-away duty (overcome stiction)
    constexpr uint8_t  PWM_MAX       = 255;  // full duty
    constexpr float    PWM_FREQUENCY_HZ = 20000.0f; // above audible range
    constexpr float    FBK_ALPHA     = 0.30f;// EMA smoothing factor

    constexpr uint32_t CONTROL_PERIOD_MS = 5;    // 200 Hz control loop
    constexpr uint32_t STATUS_PERIOD_MS  = 100;  // 10 Hz status TX
    constexpr uint32_t WATCHDOG_MS       = 100;  // tightest budget (brake first)

    // Calibration: drive the actuator extend<->retract while printing raw ADC,
    // so we can read car2's true released/applied endpoints. Set false when done.
    constexpr bool     BRAKE_CAL   = false;  // normal operation
    constexpr uint32_t CAL_HALF_MS = 3000;   // drive each direction 3 s
    constexpr uint8_t  CAL_PWM     = 255;    // FULL drive (12 V) to rule out weak-drive
}

enum : uint8_t { MOTION_EXTEND = 7, MOTION_RETRACT = 8, MOTION_STOP = 9 };

// ------------------------------- Globals ---------------------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
DbwRxState rx;

static bool  cmd_estop = true;     // start safe (brake applied)
static float cmd_brake = 1.0f;     // normalized 0..1
static float fbk_filt  = (float)cfg::ADC_RELEASED;
static uint8_t motion  = MOTION_STOP;
static uint8_t pwm_mag = 0;

static uint32_t last_valid_cmd_ms = 0;
static bool     stale = true;

// ------------------------------- Helpers ---------------------------------
static inline float clamp01(float v) { return (v < 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }

static void driveMotor(uint8_t magnitude, bool extend) {
    if (extend) {
        analogWrite(cfg::PIN_RETRACT, 0);
        analogWrite(cfg::PIN_EXTEND, magnitude);
    } else {
        analogWrite(cfg::PIN_EXTEND, 0);
        analogWrite(cfg::PIN_RETRACT, magnitude);
    }
}

static void stopMotor() {
    analogWrite(cfg::PIN_RETRACT, 0);
    analogWrite(cfg::PIN_EXTEND, 0);
}

// ------------------------------ Control ----------------------------------
static void controlStep() {
    // Filter feedback.
    const int raw = analogRead(cfg::PIN_FBK);
    fbk_filt += (float)((float)raw - fbk_filt) * cfg::FBK_ALPHA;

    // Target: full brake on any unsafe condition, else the commanded level.
    const bool  safe   = cmd_estop || stale;
    const float target = safe ? 1.0f : clamp01(cmd_brake);
    const float targetADC =
        (float)cfg::ADC_RELEASED + (float)(cfg::ADC_APPLIED - cfg::ADC_RELEASED) * target;

    const float error = targetADC - fbk_filt;

    if (fabsf(error) <= (float)cfg::DEADBAND_ADC) {
        stopMotor();
        motion  = MOTION_STOP;
        pwm_mag = 0;
        return;
    }

    // Proportional magnitude, clamped to [break-away, full].
    float mag = fabsf(error) * cfg::KP;
    if (mag < (float)cfg::PWM_MIN) mag = (float)cfg::PWM_MIN;
    if (mag > (float)cfg::PWM_MAX) mag = (float)cfg::PWM_MAX;
    pwm_mag = (uint8_t)mag;

    // Feedback decreases as the actuator extends, so a negative error applies.
    const bool extend = (error < 0.0f);
    driveMotor(pwm_mag, extend);
    motion = extend ? MOTION_EXTEND : MOTION_RETRACT;
}

// ---------------------------- CAN handling -------------------------------
static void handleCan() {
    CAN_message_t msg;
    while (can.read(msg)) {
        if (msg.id != DBW_ID_BRAKE_CMD) {
            continue;
        }
        uint8_t p[2];
        if (dbw_unpack_cmd(msg, DBW_ID_BRAKE_CMD, rx, p, 2)) {
            cmd_estop = (p[0] != 0u);
            cmd_brake = clamp01((float)p[1] / 255.0f);
            last_valid_cmd_ms = millis();
        }
    }
}

static void sendStatus() {
    const int raw = (int)(fbk_filt + 0.5f);

    float inches = (fbk_filt - (float)cfg::ADC_RELEASED) * cfg::STROKE_IN /
                   (float)(cfg::ADC_APPLIED - cfg::ADC_RELEASED);
    if (inches < 0.0f)             inches = 0.0f;
    if (inches > cfg::STROKE_IN)   inches = cfg::STROKE_IN;
    const int16_t lenCenti = (int16_t)lroundf(inches * 100.0f);

    uint8_t flags = 0u;
    if (raw >= cfg::ADC_RELEASED - cfg::DEADBAND_ADC) flags |= 0x01u;  // near released
    if (raw <= cfg::ADC_APPLIED  + cfg::DEADBAND_ADC) flags |= 0x02u;  // near applied
    if (stale)     flags |= 0x04u;
    if (cmd_estop) flags |= 0x08u;

    CAN_message_t m;
    m.id  = DBW_ID_BRAKE_STAT;
    m.len = 8;
    m.buf[0] = (uint8_t)(raw & 0xFFu);            // raw ADC (LE)
    m.buf[1] = (uint8_t)((raw >> 8) & 0xFFu);
    m.buf[2] = (uint8_t)(lenCenti & 0xFFu);       // length*100 in (LE)
    m.buf[3] = (uint8_t)((lenCenti >> 8) & 0xFFu);
    m.buf[4] = motion;                            // 7/8/9
    m.buf[5] = flags;
    m.buf[6] = pwm_mag;
    m.buf[7] = cmd_estop ? 1u : 0u;
    can.write(m);
}

// --------------------------- Calibration ---------------------------------
// Oscillates extend/retract and prints the raw ADC with running min/max so we
// can read the true endpoints on car2. Watch for stall/heat at the ends.
static void runBrakeCal(uint32_t now) {
    static uint32_t t0 = 0;
    static bool extending = true;
    static int adc_min = 1023, adc_max = 0;

    if (now - t0 >= cfg::CAL_HALF_MS) {
        t0 = now;
        extending = !extending;
    }
    driveMotor(cfg::CAL_PWM, extending);

    static uint32_t last_p = 0;
    if (now - last_p >= 100) {
        last_p = now;
        const int raw = analogRead(cfg::PIN_FBK);
        if (raw < adc_min) adc_min = raw;
        if (raw > adc_max) adc_max = raw;
        Serial.print("BRAKE CAL dir="); Serial.print(extending ? "EXTEND " : "RETRACT");
        Serial.print("  raw=");  Serial.print(raw);
        Serial.print("  min=");  Serial.print(adc_min);
        Serial.print("  max=");  Serial.println(adc_max);
    }
}

// ------------------------------ Setup/loop -------------------------------
void setup() {
    Serial.begin(115200);

    pinMode(cfg::PIN_RETRACT, OUTPUT);
    pinMode(cfg::PIN_EXTEND, OUTPUT);
    pinMode(cfg::PIN_FBK, INPUT);
    analogWriteResolution(8);
    analogWriteFrequency(cfg::PIN_RETRACT, cfg::PWM_FREQUENCY_HZ);
    analogWriteFrequency(cfg::PIN_EXTEND, cfg::PWM_FREQUENCY_HZ);
    analogReadResolution(10);
    stopMotor();

    fbk_filt = (float)analogRead(cfg::PIN_FBK);

    can.begin();
    can.setBaudRate(DBW_CAN_BITRATE);

    last_valid_cmd_ms = millis();
    Serial.println("BRAKE node up @ 250k");
}

void loop() {
    const uint32_t now = millis();

    if (cfg::BRAKE_CAL) {          // calibration mode: oscillate + print raw ADC
        runBrakeCal(now);
        return;
    }

    handleCan();
    stale = (now - last_valid_cmd_ms > cfg::WATCHDOG_MS);

    static uint32_t last_ctrl = 0;
    if (now - last_ctrl >= cfg::CONTROL_PERIOD_MS) {
        last_ctrl = now;
        controlStep();
    }

    static uint32_t last_stat = 0;
    if (now - last_stat >= cfg::STATUS_PERIOD_MS) {
        last_stat = now;
        sendStatus();
    }
}
