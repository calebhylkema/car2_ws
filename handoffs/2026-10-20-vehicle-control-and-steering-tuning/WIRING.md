# 2026-10-20 — Recorded wiring and protocol

This is the firmware pin contract and recorded operator wiring, not a continuity measurement of every wire. Pin numbers are Teensy 4.1 digital numbers. Verify actual connectors before changing hardware.

| Node | Function | Teensy pin / connection |
|---|---|---|
| All four | CAN1 TX | **22 → transceiver TXD/TX** |
| All four | CAN1 RX | **23 ← transceiver RXD/RX** |
| All four | CAN module supply | 3.3 V → module 3.3 V; common GND |
| Steering | CL57T STEP / DIR | 15 / 16; software direction inverted |
| Steering | Physically right-mounted NC switch | 17; **leftward wheel-travel guard** |
| Steering | Physically left-mounted NC switch | 18; **rightward wheel-travel guard** |
| Steering | Angle sensor ADC | **A0 = digital 14**, analog divider output |
| Steering | Driver enable | Firmware enable-pin control disabled; pin 20 is only a dormant option |
| Throttle | MCP4725 I2C | 18 SDA / 19 SCL; address 0x62, fallback 0x60 |
| Throttle | Gear B / C / D | 33 / 34 / 35 |
| Throttle | Wheel-speed quadrature A / B | 2 / 3 |
| Brake | H-bridge retract / release | 19 |
| Brake | H-bridge extend / apply | 18 |
| Brake | Actuator position feedback | A0 = 14 |

The CAN module's right screw terminals are **CAN-H and CAN-L**, the differential bus. They do not correspond individually to logic TX/RX on its left header. CAN-H joins every module's H; CAN-L joins every module's L. TXD and RXD are the separate local Teensy-to-transceiver logic connection. No software claim here establishes a specific module's terminal order: read its silkscreen.

Recorded cable order: throttle → master → steering → brake. Use one 120 Ω termination between H/L at **each physical cable end**, not at every node. With all power and USB disconnected, two 120 Ω terminations in parallel normally measure approximately 60 Ω between H/L. An earlier operator reading was approximately 30 Ω; this package does not contain a subsequent verified resistance measurement. A disconnected Teensy is not a termination resistor. An unused transceiver still on H/L can load the bus; removing a node at a cable end must preserve proper termination. Common reference and transceiver power also matter.

When external vehicle VIN and USB are both connected, provide the Teensy VIN/VUSB isolation required for the actual power arrangement. USB serial enumeration does not establish electrical power isolation.

The steering switches use INPUT_PULLUP with **NC released = LOW; pressed/open circuit = HIGH**. Manual holds established physical switch identity; wheel motion established that rightward travel approaches the physically left switch. During referenced, active operation a single switch overrides input and returns the wheels toward the existing Z, moving away from the pressed end. Steering input must return to zero before normal steering resumes. E-stop holds position; timeout, both switches, or a switch inconsistent with the recorded side stop motion and invalidate reference. The selected software limits leave clearance before those contacts. They are not measured switch-trip points and are not angles in degrees.

Operational steering speed ceiling is 24000 pulses/s with 51200 pulses/s² acceleration, 10 µs direction setup, and 25 µs STEP high time. The speed ceiling is 1.6 times the previous 15000 pulses/s; acceleration is 2.56 times the previous 20000 pulses/s² to scale the ideal motion profile by 1.6 over the same travel. Z and the -8250/+10000 pulse travel boundaries are unchanged.

Throttle uses MCP4725, not MCP4728: nominal idle 0.80 V, configured full-scale 3.30 V, software throttle ceiling 0.85. Gear pin levels B/C/D: N=H/H/H, D=L/H/H, S=L/H/L, R=H/L/H. The DAC output is the VOTOL EM-70 throttle input; it is not the separate rotary encoder. DAC readback verifies register state, not the measured voltage at the controller or controller enable state. A speed encoder fault and VOTOL motor Hall/commutation wiring are different problems.

Manual USB pulses on 2026-11-02 confirmed GPIO19 retracts/releases and GPIO18 extends/applies. The operator measured raw74 released, first grab around127-135, raw277 firmly applied, and raw564 at full actuator travel. Feedback increases during application.

Brake constants are ADC74 released, ADC277 commanded applied, and ADC564 reported full actuator travel. Normal control uses continuous full-duty drive in the released-to-working-applied range, with a 4-count stopping deadband. ADC_APPLIED=277 is provisional; retune after checking actual braking. There is no diagnostic pulse/cooldown in normal control. Length telemetry estimates nominal actuator travel using the full74-564 span. These are car-specific position values, not a guarantee of stopping force.

## Protocol compatibility

All normal environments compile the same `dbw_can.h` with `DBW_E2E_ENABLE=1`, CAN1 **250 kbit/s**. Commands include sequence and CRC checking. Keep master and nodes on this matched protocol; mixing older images can stop reception even when wiring is correct.

| Command | Status | Meaning |
|---|---|---|
| 0x100 | 0x101 | Throttle / gear |
| **0x210** | 0x201 | Steering V2 with explicit center/reference flags |
| 0x300 | 0x301 | Brake |

Legacy steering command 0x200 is deliberately rejected by current normal steering. Do not substitute old commands to get around missing reference. Master USB is 115200 baud; its parser and JSON telemetry are in `master.cpp`, host serialization in `teleop/teensy_serial.py`. The supplied controller maintains periodic commands. A one-shot manual serial line will time out: master watchdog 500 ms, throttle/steering 150 ms, brake 100 ms. Master reports `tr/sr/br` freshness (300 ms), `sh` steering reference, `sf` steering flags, and `sp` normalized **software step position**, not an independently measured wheel angle.

The initial command state is E-stop, throttle zero, N, brake applied. Brake command over 0.05 inhibits throttle. Direction reversal has neutral/brake dwell and release checks. Diagnostic CAN images speak their own test protocol and must be replaced before normal driving.
