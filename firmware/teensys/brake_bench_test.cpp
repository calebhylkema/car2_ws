// Temporary USB-controlled brake actuator diagnostic for Teensy 4.1.
// This deliberately ignores CAN and position feedback. It is not vehicle firmware.

#include <Arduino.h>

namespace cfg {
    constexpr uint8_t PIN_RETRACT = 19;
    constexpr uint8_t PIN_EXTEND = 18;
    constexpr uint8_t PIN_FEEDBACK = A0;

    constexpr uint8_t PULSE_PWM = 96;          // 38% duty at 8-bit resolution
    constexpr float PWM_FREQUENCY_HZ = 20000.0f;
    constexpr uint32_t SHORT_PULSE_MS = 200;   // fine positioning
    constexpr uint32_t LONG_PULSE_MS = 1000;   // endpoint calibration
    constexpr uint32_t COOLDOWN_MS = 1000;     // prevents queued/repeated pulses
}

enum class Motion : uint8_t { Stopped, Apply, Release };

static Motion motion = Motion::Stopped;
static uint32_t pulse_started_ms = 0;
static uint32_t pulse_duration_ms = 0;
static uint32_t next_pulse_ms = 0;

static void stopMotor() {
    analogWrite(cfg::PIN_RETRACT, 0);
    analogWrite(cfg::PIN_EXTEND, 0);
    motion = Motion::Stopped;
}

static void startPulse(Motion requested, uint32_t now, uint32_t duration_ms) {
    if (motion != Motion::Stopped || now < next_pulse_ms) {
        Serial.println(F("Pulse ignored: wait for STOP/cooldown."));
        return;
    }

    stopMotor();
    if (requested == Motion::Apply) {
        analogWrite(cfg::PIN_EXTEND, cfg::PULSE_PWM);
        Serial.print(F("APPLY pulse: pin 18 active, pin 19 off, ms="));
    } else {
        analogWrite(cfg::PIN_RETRACT, cfg::PULSE_PWM);
        Serial.print(F("RELEASE pulse: pin 19 active, pin 18 off, ms="));
    }
    Serial.println(duration_ms);

    motion = requested;
    pulse_started_ms = now;
    pulse_duration_ms = duration_ms;
    next_pulse_ms = now + duration_ms + cfg::COOLDOWN_MS;
}

static void printHelp() {
    Serial.println();
    Serial.println(F("Brake bench diagnostic (not vehicle firmware)"));
    Serial.println(F("  a = one 200 ms APPLY pulse on pin 18"));
    Serial.println(F("  r = one 200 ms RELEASE pulse on pin 19"));
    Serial.println(F("  A = one 1000 ms APPLY calibration pulse"));
    Serial.println(F("  R = one 1000 ms RELEASE calibration pulse"));
    Serial.println(F("  x = stop immediately"));
    Serial.println(F("  ? = show this help"));
}

void setup() {
    pinMode(cfg::PIN_RETRACT, OUTPUT);
    pinMode(cfg::PIN_EXTEND, OUTPUT);
    pinMode(cfg::PIN_FEEDBACK, INPUT);
    analogWriteResolution(8);
    analogWriteFrequency(cfg::PIN_RETRACT, cfg::PWM_FREQUENCY_HZ);
    analogWriteFrequency(cfg::PIN_EXTEND, cfg::PWM_FREQUENCY_HZ);
    analogReadResolution(10);
    stopMotor();

    Serial.begin(115200);
    delay(500);
    printHelp();
}

void loop() {
    const uint32_t now = millis();

    if (motion != Motion::Stopped && now - pulse_started_ms >= pulse_duration_ms) {
        stopMotor();
        Serial.println(F("STOP"));
    }

    while (Serial.available()) {
        const char command = (char)Serial.read();
        if (command == 'a') {
            startPulse(Motion::Apply, now, cfg::SHORT_PULSE_MS);
        } else if (command == 'r') {
            startPulse(Motion::Release, now, cfg::SHORT_PULSE_MS);
        } else if (command == 'A') {
            startPulse(Motion::Apply, now, cfg::LONG_PULSE_MS);
        } else if (command == 'R') {
            startPulse(Motion::Release, now, cfg::LONG_PULSE_MS);
        } else if (command == 'x') {
            stopMotor();
            next_pulse_ms = now + cfg::COOLDOWN_MS;
            Serial.println(F("STOP requested"));
        } else if (command == '?') {
            printHelp();
        }
    }

    static uint32_t last_status_ms = 0;
    if (now - last_status_ms >= 250) {
        last_status_ms = now;
        const int raw = analogRead(cfg::PIN_FEEDBACK);
        const float volts = (float)raw * 3.3f / 1023.0f;
        Serial.print(F("feedback raw="));
        Serial.print(raw);
        Serial.print(F(" volts="));
        Serial.println(volts, 3);
    }
}
