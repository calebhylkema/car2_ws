// master.cpp — car2 drive-by-wire MASTER bridge (Teensy 4.1)
//
// Bridges the PC (USB CDC serial @115200) to the actuator nodes (CAN1 @250k).
// The PC-side line protocol and JSON telemetry are the frozen contract in
// PROTOCOL.md, so the keyboard test tool today and a ROS2 node tomorrow need no
// changes.
//
// Design highlights (see DESIGN.md §5):
//   - non-blocking fixed-rate tasks: serial parse, 100 Hz CAN TX, 10 Hz telemetry
//   - outgoing command frames carry E2E integrity (counter+CRC) via dbw_can.h
//   - brake/throttle plausibility interlock: any real brake forces throttle to 0
//   - 500 ms serial watchdog -> estop; each node also self-fails independently
//
// Downlink: E T M B S C A P (+ '?' debug).  Uplink: JSON telemetry line @10 Hz.

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "dbw_can.h"

// ----------------------------- Configuration -----------------------------
namespace cfg {
    constexpr uint32_t CAN_TX_PERIOD_MS   = 10;    // 100 Hz command TX
    constexpr uint32_t TELEM_PERIOD_MS    = 100;   // 10 Hz telemetry
    constexpr uint32_t WATCHDOG_MS        = 500;   // PC comms timeout -> estop
    constexpr float    BRAKE_INTERLOCK    = 0.05f; // brake above this zeroes throttle
    constexpr float    THROTTLE_CEILING   = 0.75f; // vehicle-wide maximum command
    constexpr float    RELEASE_THRESHOLD  = 0.01f; // released accelerator command
    constexpr float    FULL_BRAKE_LEVEL   = 0.95f;
    constexpr uint32_t DIRECTION_GUARD_MS = 1000;  // Neutral + brake dwell
    constexpr uint32_t THROTTLE_STATUS_TIMEOUT_MS = 300;
}

// ------------------------------- Globals ---------------------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

// Commanded vehicle state
static bool  g_estop    = false;
static float g_throttle = 0.0f;
static char  g_mode     = 'N';
static float g_brake    = 0.0f;
static float g_steer    = 0.0f;
static bool  g_center   = false;

// Per-ID E2E sequence counters
static uint8_t seq_thr = 0, seq_steer = 0, seq_brake = 0;

// Node feedback (relayed to telemetry)
static uint8_t fb_thr_gear   = 0;
static int16_t fb_thr_vcenti = 0;
static uint32_t fb_thr_status_ms = 0;
static int8_t  fb_steer_pct  = 0;
static uint8_t fb_steer_homed = 0;
static uint8_t fb_steer_flags = 0;
static int16_t fb_brake_centi = 0;
static uint8_t fb_brake_flags = 0;

// Watchdog
static uint32_t g_last_cmd = 0;
static bool     g_wd_active = false;

// Embedded D<->R guard. This remains active for every command source that uses
// the master, even when the PC-side teleop is replaced.
static bool     g_dir_guard_active = false;
static char     g_dir_guard_target = 'N';
static char     g_last_motion_gear = 'N';
static char     g_guard_bypass_gear = 'N';
static bool     g_dir_release_seen = false;
static bool     g_neutral_brake_satisfied = false;
static uint32_t g_dir_guard_started_ms = 0;
static uint32_t g_neutral_brake_started_ms = 0;

// ------------------------------- Helpers ---------------------------------
static inline float clamp01(float v) { return (v < 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }
static inline float clamp11(float v) { return (v < -1.0f) ? -1.0f : (v > 1.0f ? 1.0f : v); }

static uint8_t toU8(float v) { return (uint8_t)(clamp01(v) * 255.0f + 0.5f); }
static int8_t  toI8(float v) {
    const float s = clamp11(v) * 127.0f;
    return (int8_t)(s + (s >= 0.0f ? 0.5f : -0.5f));
}

static bool isDirectionGear(char mode) { return mode == 'D' || mode == 'R'; }

static char feedbackGearChar() {
    switch (fb_thr_gear) {
        case 1: return 'D';
        case 2: return 'S';
        case 3: return 'R';
        default: return 'N';
    }
}

static bool throttleFeedbackFresh(uint32_t now) {
    return fb_thr_status_ms != 0u &&
           now - fb_thr_status_ms <= cfg::THROTTLE_STATUS_TIMEOUT_MS;
}

static bool feedbackMatches(char mode) {
    return feedbackGearChar() == mode;
}

static void startDirectionGuard(char target, uint32_t now) {
    g_dir_guard_active = true;
    g_dir_guard_target = target;
    g_dir_release_seen = false;
    g_dir_guard_started_ms = now;
    g_guard_bypass_gear = 'N';
}

static void resetDirectionGuard() {
    g_dir_guard_active = false;
    g_dir_guard_target = 'N';
    g_dir_release_seen = false;
    g_dir_guard_started_ms = 0;
    g_guard_bypass_gear = 'N';
    g_neutral_brake_satisfied = false;
    g_neutral_brake_started_ms = 0;
}

// ---------------------------- CAN command TX -----------------------------
static void sendCommands() {
    const uint32_t now = millis();
    float eff_throttle = clamp01(g_throttle);
    float eff_brake = clamp01(g_brake);
    char eff_mode = g_mode;

    if (g_estop) {
        eff_throttle = 0.0f;
        eff_brake = 1.0f;
        eff_mode = 'N';
        resetDirectionGuard();
    } else {
        const bool feedback_fresh = throttleFeedbackFresh(now);
        const char actual_gear = feedbackGearChar();

        if (feedback_fresh && isDirectionGear(actual_gear)) {
            if (actual_gear != g_last_motion_gear) {
                g_last_motion_gear = actual_gear;
                g_neutral_brake_satisfied = false;
                g_neutral_brake_started_ms = 0;
            }
            if (g_guard_bypass_gear == actual_gear) {
                g_guard_bypass_gear = 'N';
            }
        }

        // Recognize a completed PC-side guard so the embedded guard does not
        // add a second delay after Neutral + full brake has already been held.
        const bool neutral_brake_commanded =
            g_mode == 'N' && g_throttle <= cfg::RELEASE_THRESHOLD &&
            g_brake >= cfg::FULL_BRAKE_LEVEL;
        if (neutral_brake_commanded) {
            if (g_neutral_brake_started_ms == 0u) {
                g_neutral_brake_started_ms = now;
            }
            if (now - g_neutral_brake_started_ms >= cfg::DIRECTION_GUARD_MS &&
                feedback_fresh && actual_gear == 'N') {
                g_neutral_brake_satisfied = true;
            }
        } else if (g_mode == 'N') {
            g_neutral_brake_started_ms = 0;
            g_neutral_brake_satisfied = false;
        }

        if (g_guard_bypass_gear != 'N' && g_mode != g_guard_bypass_gear) {
            g_guard_bypass_gear = 'N';
        }

        if (!g_dir_guard_active && isDirectionGear(g_mode) &&
            isDirectionGear(g_last_motion_gear) &&
            g_mode != g_last_motion_gear &&
            g_guard_bypass_gear != g_mode) {
            if (g_neutral_brake_satisfied) {
                g_guard_bypass_gear = g_mode;
                g_neutral_brake_satisfied = false;
                g_neutral_brake_started_ms = 0;
            } else {
                startDirectionGuard(g_mode, now);
            }
        }

        bool hold_for_guard = false;
        if (g_dir_guard_active) {
            // Returning to the previous direction cancels the requested swap.
            if (g_mode == g_last_motion_gear) {
                g_dir_guard_active = false;
                g_dir_guard_target = 'N';
                g_dir_release_seen = false;
            } else {
                if (isDirectionGear(g_mode) && g_mode != g_dir_guard_target) {
                    startDirectionGuard(g_mode, now);
                }

                const bool released_now =
                    g_mode == 'N' || g_throttle <= cfg::RELEASE_THRESHOLD;
                if (released_now) {
                    g_dir_release_seen = true;
                }

                const bool guard_complete =
                    now - g_dir_guard_started_ms >= cfg::DIRECTION_GUARD_MS &&
                    g_dir_release_seen && released_now && feedback_fresh &&
                    actual_gear == 'N';

                hold_for_guard = true;
                if (guard_complete) {
                    g_dir_guard_active = false;
                    if (g_mode == g_dir_guard_target && isDirectionGear(g_mode)) {
                        g_guard_bypass_gear = g_mode;
                    } else {
                        g_neutral_brake_satisfied = true;
                    }
                    g_dir_guard_target = 'N';
                    g_dir_release_seen = false;
                }
            }
        }

        if (hold_for_guard) {
            eff_throttle = 0.0f;
            eff_brake = 1.0f;
            eff_mode = 'N';
        } else {
            if (eff_throttle > cfg::THROTTLE_CEILING) {
                eff_throttle = cfg::THROTTLE_CEILING;
            }
            if (eff_mode == 'N') {
                eff_throttle = 0.0f;
            } else if (!feedback_fresh || !feedbackMatches(eff_mode)) {
                // Gear commands may proceed, but torque waits for fresh proof
                // that the throttle node has reached the requested gear.
                eff_throttle = 0.0f;
            }
        }
    }

    // Plausibility interlock: a real brake command cancels throttle.
    if (eff_brake > cfg::BRAKE_INTERLOCK) {
        eff_throttle = 0.0f;
    }

    CAN_message_t m;

    uint8_t thr[3] = { (uint8_t)(g_estop ? 1u : 0u), toU8(eff_throttle), (uint8_t)eff_mode };
    dbw_pack_cmd(m, DBW_ID_THROTTLE_CMD, seq_thr, thr, 3);
    can.write(m);

    uint8_t brk[2] = { (uint8_t)(g_estop ? 1u : 0u), toU8(eff_brake) };
    dbw_pack_cmd(m, DBW_ID_BRAKE_CMD, seq_brake, brk, 2);
    can.write(m);

    uint8_t str[2] = { (uint8_t)(g_center ? 1u : 0u), (uint8_t)toI8(g_steer) };
    dbw_pack_cmd(m, DBW_ID_STEER_CMD, seq_steer, str, 2);
    can.write(m);
    g_center = false;  // one-shot
}

// ---------------------------- CAN status RX ------------------------------
static void handleStatus() {
    CAN_message_t msg;
    while (can.read(msg)) {
        switch (msg.id) {
            case DBW_ID_THROTTLE_STAT:
                if (msg.len >= 6) {
                    fb_thr_gear   = msg.buf[2];
                    fb_thr_vcenti = (int16_t)(msg.buf[4] | ((uint16_t)msg.buf[5] << 8));
                    fb_thr_status_ms = millis();
                }
                break;
            case DBW_ID_STEER_STAT:
                if (msg.len >= 8) {
                    fb_steer_pct   = (int8_t)msg.buf[4];
                    fb_steer_flags = msg.buf[5];
                    fb_steer_homed = msg.buf[7];
                }
                break;
            case DBW_ID_BRAKE_STAT:
                if (msg.len >= 6) {
                    fb_brake_centi = (int16_t)(msg.buf[2] | ((uint16_t)msg.buf[3] << 8));
                    fb_brake_flags = msg.buf[5];
                }
                break;
            default:
                break;
        }
    }
}

// ---------------------------- Telemetry TX -------------------------------
static void sendTelemetry() {
    char buf[192];
    snprintf(buf, sizeof(buf),
        "{\"e\":%d,\"t\":%.3f,\"m\":\"%c\",\"b\":%.3f,\"s\":%.3f,\"w\":%d,"
        "\"tv\":%.2f,\"tm\":%u,\"sp\":%d,\"sh\":%u,\"sf\":%u,\"bl\":%.2f,\"bf\":%u}",
        g_estop ? 1 : 0, g_throttle, g_mode, g_brake, g_steer, g_wd_active ? 1 : 0,
        (double)fb_thr_vcenti / 100.0, (unsigned)fb_thr_gear,
        (int)fb_steer_pct, (unsigned)fb_steer_homed, (unsigned)fb_steer_flags,
        (double)fb_brake_centi / 100.0, (unsigned)fb_brake_flags);
    Serial.println(buf);
}

// ---------------------------- Command parsing ----------------------------
static void parseAll(char* args) {
    char* save = nullptr;
    for (char* tok = strtok_r(args, " ,\t", &save); tok; tok = strtok_r(nullptr, " ,\t", &save)) {
        if (tok[0] == '\0' || (tok[1] != '=' && tok[1] != ':')) {
            continue;
        }
        const char key = (char)toupper((unsigned char)tok[0]);
        const char* val = tok + 2;
        switch (key) {
            case 'E': g_estop = (atoi(val) != 0); break;
            case 'T': g_throttle = clamp01((float)atof(val)); break;
            case 'M': { char c = (char)toupper((unsigned char)val[0]);
                        if (c=='N'||c=='D'||c=='S'||c=='R') g_mode = c; } break;
            case 'B': g_brake = clamp01((float)atof(val)); break;
            case 'S': g_steer = clamp11((float)atof(val)); break;
            case 'C': if (atoi(val) != 0) { g_center = true; g_steer = 0.0f; } break;
            default: break;
        }
    }
}

static void parseLine(char* line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') return;

    const char cmd = (char)toupper((unsigned char)*line++);
    while (*line == ' ' || *line == '\t') ++line;

    switch (cmd) {
        case 'E': g_estop = (atoi(line) != 0); break;
        case 'T': g_throttle = clamp01((float)atof(line)); break;
        case 'M': { char c = (char)toupper((unsigned char)*line);
                    if (c=='N'||c=='D'||c=='S'||c=='R') g_mode = c; } break;
        case 'B': g_brake = clamp01((float)atof(line)); break;
        case 'S': g_steer = clamp11((float)atof(line)); break;
        case 'C': g_center = true; g_steer = 0.0f; break;
        case 'A': parseAll(line); break;
        case 'P': sendTelemetry(); return;   // query only; no watchdog feed
        default: return;
    }
    g_last_cmd  = millis();
    g_wd_active = true;
}

static void handleSerial() {
    static char buf[128];
    static uint8_t idx = 0;

    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (idx > 0) {
                buf[idx] = '\0';
                if (buf[0] == '?') {
                    Serial.print(F("E=")); Serial.print(g_estop);
                    Serial.print(F(" T=")); Serial.print(g_throttle, 3);
                    Serial.print(F(" M=")); Serial.print(g_mode);
                    Serial.print(F(" B=")); Serial.print(g_brake, 3);
                    Serial.print(F(" S=")); Serial.print(g_steer, 3);
                    Serial.print(F(" W=")); Serial.println(g_wd_active);
                } else {
                    parseLine(buf);
                }
                idx = 0;
            }
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

// ------------------------------ Watchdog ---------------------------------
static void checkWatchdog() {
    if (!g_wd_active) return;
    if (millis() - g_last_cmd > cfg::WATCHDOG_MS) {
        if (!g_estop) {
            g_estop = true;
            g_throttle = 0.0f;
            g_brake = 1.0f;
            Serial.println(F("WATCHDOG -> ESTOP"));
        }
    }
}

// ------------------------------ Setup/loop -------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("=== car2 CAN Master (USB serial) ==="));

    can.begin();
    can.setBaudRate(DBW_CAN_BITRATE);

    g_last_cmd = millis();
    Serial.println(F("Ready."));
}

void loop() {
    handleSerial();
    handleStatus();
    checkWatchdog();

    const uint32_t now = millis();

    static uint32_t last_tx = 0;
    if (now - last_tx >= cfg::CAN_TX_PERIOD_MS) {
        last_tx = now;
        sendCommands();
    }

    static uint32_t last_telem = 0;
    if (now - last_telem >= cfg::TELEM_PERIOD_MS) {
        last_telem = now;
        sendTelemetry();
    }
}
