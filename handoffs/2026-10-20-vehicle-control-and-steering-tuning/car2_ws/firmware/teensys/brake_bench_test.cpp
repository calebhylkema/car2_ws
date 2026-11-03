// Temporary USB-only hold-to-move brake diagnostic. Not vehicle firmware.
// Use brake_usb_hold_test.py for key release detection and periodic commands.
#include <Arduino.h>
namespace cfg {
constexpr uint8_t PIN_RETRACT = 19, PIN_EXTEND = 18, PIN_FEEDBACK = A0;
constexpr uint32_t WATCHDOG_MS = 150;
constexpr int RELEASED_LIMIT = 74, APPLIED_LIMIT = 564;
}
enum class Motion : uint8_t { Stopped, Apply, Release };
static Motion requested = Motion::Stopped, motion = Motion::Stopped;
static bool limit_latched = false;
static uint32_t last_command_ms = 0;
static void stopMotor() {
    analogWrite(cfg::PIN_RETRACT, 0);
    analogWrite(cfg::PIN_EXTEND, 0);
    motion = Motion::Stopped;
}
static void requestMotion(Motion next, uint32_t now) {
    if (next != requested || next == Motion::Stopped) limit_latched = false;
    requested = next;
    last_command_ms = now;
    if (next == Motion::Stopped) stopMotor();
}
static void printHelp() {
    Serial.println(F("BRAKE_USB_HOLD_TEST_V1"));
    Serial.println(F("Keyboard helper: hold A/Space=apply pin18, R=release pin19; release=stop; Esc=exit."));
    Serial.println(F("Full duty; timeout150ms; measured limits raw74 released / raw564 applied."));
}
void setup() {
    pinMode(cfg::PIN_RETRACT, OUTPUT); pinMode(cfg::PIN_EXTEND, OUTPUT);
    pinMode(cfg::PIN_FEEDBACK, INPUT);
    analogWriteResolution(8); analogReadResolution(10);
    analogWriteFrequency(cfg::PIN_RETRACT, 20000);
    analogWriteFrequency(cfg::PIN_EXTEND, 20000);
    stopMotor(); Serial.begin(115200);
}
void loop() {
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == 'a') requestMotion(Motion::Apply, millis());
        else if (c == 'r') requestMotion(Motion::Release, millis());
        else if (c == 'x') requestMotion(Motion::Stopped, millis());
        else if (c == '?') printHelp();
    }
    const uint32_t now = millis();
    const int raw = analogRead(cfg::PIN_FEEDBACK);
    if (!Serial || now - last_command_ms > cfg::WATCHDOG_MS || raw < 8 || raw > 1015) {
        requested = Motion::Stopped; limit_latched = false; stopMotor();
    } else if (requested == Motion::Stopped) stopMotor();
    else {
        if ((requested == Motion::Apply && raw >= cfg::APPLIED_LIMIT) ||
            (requested == Motion::Release && raw <= cfg::RELEASED_LIMIT)) limit_latched = true;
        if (limit_latched) stopMotor();
        else if (requested == Motion::Apply) {
            analogWrite(cfg::PIN_RETRACT, 0); analogWrite(cfg::PIN_EXTEND, 255); motion = requested;
        } else {
            analogWrite(cfg::PIN_EXTEND, 0); analogWrite(cfg::PIN_RETRACT, 255); motion = requested;
        }
    }
    static uint32_t last_status_ms = 0;
    if (Serial && now - last_status_ms >= 100) {
        last_status_ms = now;
        Serial.printf("feedback raw=%d motion=%s limit=%d\n", raw,
                      motion == Motion::Apply ? "APPLY" : motion == Motion::Release ? "RELEASE" : "STOP",
                      limit_latched ? 1 : 0);
    }
}
