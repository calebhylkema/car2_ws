// throttle.cpp — car2 drive-by-wire THROTTLE node (Teensy 4.1)
//
// Receives throttle + gear commands on CAN 0x100, drives an MCP4725 DAC (0-3.3 V
// into the vehicle throttle input) and three digital gear-select lines. Reports
// status on CAN 0x101.
//
// Design highlights (see DESIGN.md §2):
//   - fully non-blocking: gear changes run as a timed state machine, never delay()
//   - BARE throttle: the command maps straight to voltage, no on-node shaping or
//     limiting -- ramps/caps/curves belong at the teleop/ROS layer
//   - fail-safe (estop / stale command / rejected frame) = Neutral + idle voltage
//   - command frames validated by the shared E2E layer (dbw_can.h)
//
// Hardware truth (unchanged from the vehicle wiring): MCP4725 @ I2C 0x62 (fallback
// 0x60); idle 0.80 V, max 3.30 V; gear lines on pins 33/34/35.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Encoder.h>
#include "dbw_can.h"

// ----------------------------- Configuration -----------------------------
namespace cfg {
    constexpr uint8_t  PIN_GEAR_B = 33;
    constexpr uint8_t  PIN_GEAR_C = 34;
    constexpr uint8_t  PIN_GEAR_D = 35;

    // LPD3806 quadrature encoder (A=green -> pin 2, B=white -> pin 3).
    constexpr uint8_t  PIN_ENC_A = 2;
    constexpr uint8_t  PIN_ENC_B = 3;
    constexpr float    ENC_CPR   = 2400.0f;   // counts/rev (600 pulses/rev x4 quadrature)

    constexpr float    DAC_VREF        = 3.30f;   // MCP4725 output range
    constexpr float    THROTTLE_IDLE_V = 0.80f;   // "no drive" resting voltage
    constexpr float    THROTTLE_MAX_V  = 3.30f;   // full throttle voltage

    constexpr uint32_t CONTROL_PERIOD_MS = 10;    // 100 Hz control/DAC update
    constexpr uint32_t STATUS_PERIOD_MS  = 100;   // 10 Hz status TX
    constexpr uint32_t WATCHDOG_MS       = 150;   // stale-command safe-state timeout

    // Non-blocking gear-change dwell times (were blocking delays in avl2).
    constexpr uint32_t GEAR_NEUTRAL_SETTLE_MS = 300;
    constexpr uint32_t GEAR_TARGET_SETTLE_MS  = 200;
}

// ------------------------------- Types -----------------------------------
enum class Gear : uint8_t { N = 0, D = 1, S = 2, R = 3 };

enum class ShiftPhase : uint8_t { Idle, ToNeutral, ToTarget };

// ------------------------------- Globals ---------------------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
Adafruit_MCP4725 dac;
DbwRxState rx;
Encoder throttleEnc(cfg::PIN_ENC_A, cfg::PIN_ENC_B);  // sets INPUT_PULLUP on A/B

// Commanded state (from CAN)
static bool  cmd_estop    = true;    // start safe
static float cmd_throttle = 0.0f;    // normalized 0..1
static Gear  cmd_gear     = Gear::N;

// Applied state
static Gear  cur_gear     = Gear::N;
static float applied_v    = cfg::THROTTLE_IDLE_V;

// Gear-shift state machine
static ShiftPhase shift_phase = ShiftPhase::Idle;
static Gear       shift_target = Gear::N;
static uint32_t   shift_t0    = 0;

// Timers / health
static uint32_t last_valid_cmd_ms = 0;
static bool     stale = true;

// ------------------------------- Helpers ---------------------------------
static inline float clamp01(float v) { return (v < 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }

static Gear gearFromChar(char c) {
    switch (c) {
        case 'D': return Gear::D;
        case 'S': return Gear::S;
        case 'R': return Gear::R;
        default:  return Gear::N;
    }
}

static uint16_t voltsToCounts(float v) {
    if (v < 0.0f)          v = 0.0f;
    if (v > cfg::DAC_VREF) v = cfg::DAC_VREF;
    return (uint16_t)(v * (4095.0f / cfg::DAC_VREF) + 0.5f);
}

// ------------------------- Gear-select HAL --------------------------------
// Pattern truth: N=HHH, D=LHH, S=LHL, R=HLH  (pins B,C,D).
static void writeGearLines(Gear g) {
    switch (g) {
        case Gear::N: digitalWrite(cfg::PIN_GEAR_B, HIGH); digitalWrite(cfg::PIN_GEAR_C, HIGH); digitalWrite(cfg::PIN_GEAR_D, HIGH); break;
        case Gear::D: digitalWrite(cfg::PIN_GEAR_B, LOW);  digitalWrite(cfg::PIN_GEAR_C, HIGH); digitalWrite(cfg::PIN_GEAR_D, HIGH); break;
        case Gear::S: digitalWrite(cfg::PIN_GEAR_B, LOW);  digitalWrite(cfg::PIN_GEAR_C, HIGH); digitalWrite(cfg::PIN_GEAR_D, LOW);  break;
        case Gear::R: digitalWrite(cfg::PIN_GEAR_B, HIGH); digitalWrite(cfg::PIN_GEAR_C, LOW);  digitalWrite(cfg::PIN_GEAR_D, HIGH); break;
    }
}

static void writeThrottleVolts(float v) {
    applied_v = v;
    dac.setVoltage(voltsToCounts(v), false);
}

// --------------------- Non-blocking gear shifting ------------------------
// A gear change forces Neutral + idle, waits, sets the target gear, waits, then
// resumes. The loop keeps running throughout (CAN, watchdog, status all live).
static void requestGear(Gear g) {
    if (g == cur_gear && shift_phase == ShiftPhase::Idle) {
        return;
    }
    if (shift_phase != ShiftPhase::Idle && g == shift_target) {
        return;  // already shifting there
    }
    shift_target = g;
    shift_phase  = ShiftPhase::ToNeutral;
    shift_t0     = millis();
    writeGearLines(Gear::N);
}

static bool shiftInProgress() { return shift_phase != ShiftPhase::Idle; }

static void updateShift() {
    if (shift_phase == ShiftPhase::Idle) {
        return;
    }
    const uint32_t now = millis();
    if (shift_phase == ShiftPhase::ToNeutral) {
        if (now - shift_t0 >= cfg::GEAR_NEUTRAL_SETTLE_MS) {
            writeGearLines(shift_target);
            shift_phase = ShiftPhase::ToTarget;
            shift_t0    = now;
        }
    } else {  // ToTarget
        if (now - shift_t0 >= cfg::GEAR_TARGET_SETTLE_MS) {
            cur_gear    = shift_target;
            shift_phase = ShiftPhase::Idle;
        }
    }
}

// ------------------------------ Control ----------------------------------
static void controlStep() {
    // Bare mapping: the commanded 0..1 goes straight to the throttle voltage.
    // No ramp/cap/curve here -- that shaping belongs at the teleop/ROS layer.
    float demand_v;
    const bool safe = cmd_estop || stale;
    if (safe) {
        demand_v = cfg::THROTTLE_IDLE_V;                 // fail-safe: no torque
        requestGear(Gear::N);
    } else {
        requestGear(cmd_gear);
        // Output idle mid-shift so no torque is applied while gear lines settle.
        demand_v = shiftInProgress()
                     ? cfg::THROTTLE_IDLE_V
                     : cfg::THROTTLE_IDLE_V + (cfg::THROTTLE_MAX_V - cfg::THROTTLE_IDLE_V) * clamp01(cmd_throttle);
    }
    writeThrottleVolts(demand_v);
}

// ---------------------------- CAN handling -------------------------------
static void handleCan() {
    CAN_message_t msg;
    while (can.read(msg)) {
        if (msg.id != DBW_ID_THROTTLE_CMD) {
            continue;
        }
        uint8_t p[3];
        if (dbw_unpack_cmd(msg, DBW_ID_THROTTLE_CMD, rx, p, 3)) {
            cmd_estop    = (p[0] != 0u);
            cmd_throttle = clamp01((float)p[1] / 255.0f);
            cmd_gear     = gearFromChar((char)p[2]);
            last_valid_cmd_ms = millis();
        }
    }
}

static void sendStatus() {
    const uint16_t counts = voltsToCounts(applied_v);
    const int16_t  vcenti = (int16_t)lroundf(applied_v * 100.0f);

    uint8_t flags = 0u;
    if (stale)             flags |= 0x01u;
    if (shiftInProgress()) flags |= 0x02u;
    if (cmd_estop)         flags |= 0x04u;

    CAN_message_t m;
    m.id  = DBW_ID_THROTTLE_STAT;
    m.len = 8;
    m.buf[0] = (uint8_t)(counts >> 8);
    m.buf[1] = (uint8_t)(counts & 0xFFu);
    m.buf[2] = (uint8_t)cur_gear;                 // 0=N,1=D,2=S,3=R
    m.buf[3] = flags;
    m.buf[4] = (uint8_t)(vcenti & 0xFFu);         // volts*100 (LE)
    m.buf[5] = (uint8_t)((vcenti >> 8) & 0xFFu);
    m.buf[6] = (uint8_t)shift_target;             // gear being shifted to
    m.buf[7] = 0u;
    can.write(m);
}

// ------------------------------ Setup/loop -------------------------------
void setup() {
    Serial.begin(115200);

    pinMode(cfg::PIN_GEAR_B, OUTPUT);
    pinMode(cfg::PIN_GEAR_C, OUTPUT);
    pinMode(cfg::PIN_GEAR_D, OUTPUT);
    writeGearLines(Gear::N);

    Wire.begin();
    Wire.setClock(400000u);
    if (!dac.begin(0x62) && !dac.begin(0x60)) {
        Serial.println("ERROR: MCP4725 not found");
    }
    writeThrottleVolts(cfg::THROTTLE_IDLE_V);

    can.begin();
    can.setBaudRate(DBW_CAN_BITRATE);

    last_valid_cmd_ms = millis();
    Serial.println("THROTTLE node up @ 250k");
}

void loop() {
    const uint32_t now = millis();

    handleCan();
    updateShift();

    // Command watchdog -> safe state.
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

        // Encoder readout (turn the shaft to verify): raw count + RPM.
        static long     enc_last    = 0;
        static uint32_t enc_last_ms = 0;
        const long enc_now = throttleEnc.read();
        float rpm = 0.0f;
        if (enc_last_ms != 0) {
            const float dt = (float)(now - enc_last_ms) / 1000.0f;
            if (dt > 0.0f) {
                rpm = ((float)(enc_now - enc_last) / cfg::ENC_CPR) / dt * 60.0f;
            }
        }
        enc_last    = enc_now;
        enc_last_ms = now;
        Serial.print("ENC count="); Serial.print(enc_now);
        Serial.print("  rpm=");      Serial.println(rpm, 1);
    }
}
