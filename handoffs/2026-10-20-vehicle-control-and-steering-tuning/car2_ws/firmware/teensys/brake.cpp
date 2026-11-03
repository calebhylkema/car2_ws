// brake.cpp — car2 drive-by-wire BRAKE node (Teensy 4.1)
//
// Receives brake commands on CAN 0x300, drives a linear actuator through an
// H-bridge (RPWM/LPWM) with analog position feedback, and reports on CAN 0x301.
//
// Continuous full-duty drive toward the CAN-requested feedback position.
// Stops within the target deadband; no pulse timer, cooldown or PWM ramp-down.
// Existing protected CAN frames, telemetry and E-stop/watchdog apply behavior remain.
// Full-brake position is provisional and must be retuned if vehicle testing requires it.
//
// Installed motor polarity: pin 19 retracts/releases, pin 18 extends/applies,
// and position feedback on A0 increases as the actuator extends. On 2026-11-02
// the operator measured raw74 released, first grab around127-135, and raw277
// firmly applied. Raw564 is the reported full-travel endpoint, not the brake target.

#include <Arduino.h>
#include "dbw_can.h"

// ----------------------------- Configuration -----------------------------
namespace cfg {
    constexpr uint8_t  PIN_RETRACT = 19; // retract actuator, release brake
    constexpr uint8_t  PIN_EXTEND  = 18; // extend actuator, apply brake
    constexpr uint8_t  PIN_FBK     = A0; // potentiometer wiper

    constexpr int      ADC_RELEASED = 74;    // measured full retract / brake off
    constexpr int      ADC_APPLIED_LIMIT = 564; // reported full actuator travel
    constexpr int      ADC_APPLIED  = 277;   // PROVISIONAL full-brake target; retune this value
    constexpr float    STROKE_IN     = 2.0f; // for status reporting only

    constexpr int      DEADBAND_ADC  = 4;    // stop band around target
    constexpr uint8_t  PWM_MAX       = 255;  // full duty
    constexpr float    PWM_FREQUENCY_HZ = 20000.0f; // above audible range

    constexpr uint32_t CONTROL_PERIOD_MS = 5;    // 200 Hz control loop
    constexpr uint32_t STATUS_PERIOD_MS  = 100;  // 10 Hz status TX
    constexpr uint32_t WATCHDOG_MS       = 100;  // tightest budget (brake first)


}

enum : uint8_t { MOTION_EXTEND = 7, MOTION_RETRACT = 8, MOTION_STOP = 9 };

// ------------------------------- Globals ---------------------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
DbwRxState rx;

static bool  cmd_estop = true;     // start safe (brake applied)
static float cmd_brake = 1.0f;     // normalized 0..1
static int feedback_raw = cfg::ADC_RELEASED;
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
    feedback_raw = analogRead(cfg::PIN_FBK);

    // Target: full brake on any unsafe condition, else the commanded level.
    const bool  safe   = cmd_estop || stale;
    const float target = safe ? 1.0f : clamp01(cmd_brake);
    const float targetADC =
        (float)cfg::ADC_RELEASED + (float)(cfg::ADC_APPLIED - cfg::ADC_RELEASED) * target;

    const float error = targetADC - feedback_raw;

    if (fabsf(error) <= (float)cfg::DEADBAND_ADC) {
        stopMotor();
        motion  = MOTION_STOP;
        pwm_mag = 0;
        return;
    }

    pwm_mag = cfg::PWM_MAX; // same continuous full drive as the verified USB test

    // Feedback increases as the actuator extends, so a positive error applies.
    const bool extend = (error > 0.0f);
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
    const int raw = feedback_raw;

    float inches = (feedback_raw - (float)cfg::ADC_RELEASED) * cfg::STROKE_IN /
                   (float)(cfg::ADC_APPLIED_LIMIT - cfg::ADC_RELEASED);
    if (inches < 0.0f)             inches = 0.0f;
    if (inches > cfg::STROKE_IN)   inches = cfg::STROKE_IN;
    const int16_t lenCenti = (int16_t)lroundf(inches * 100.0f);

    uint8_t flags = 0u;
    if (raw <= cfg::ADC_RELEASED + cfg::DEADBAND_ADC) flags |= 0x01u;  // near released
    if (raw >= cfg::ADC_APPLIED  - cfg::DEADBAND_ADC) flags |= 0x02u;  // near applied
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

    feedback_raw = analogRead(cfg::PIN_FBK);

    can.begin();
    can.setBaudRate(DBW_CAN_BITRATE);

    last_valid_cmd_ms = millis();
    Serial.println("BRAKE node up @ 250k");
}

void loop() {
    handleCan();
    // Sample after CAN timestamps the accepted command, including across wrap.
    const uint32_t now = millis();
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
