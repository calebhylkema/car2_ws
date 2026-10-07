# car2_ws

Teensy 4.1 firmware and a Windows keyboard controller for the `car2` drive-by-wire test vehicle.

## Current Status

Last updated: **2026-10-06 21:42 PDT (UTC-07:00)**

| System | Status | Notes |
| --- | --- | --- |
| Master Teensy and CAN link | Working; latest master flashed | USB serial at 115200 baud and vehicle CAN at 250 kbit/s are working. The 2026-10-06 master with startup E-stop and embedded direction guarding has been flashed. |
| Throttle and gear control | Working after wiring repair | A wiring fault caused the apparent software regression. The fault was corrected after restoring the known-working master and throttle firmware. The newer 75% firmware ceilings remain ready for a separate controlled vehicle test. |
| Steering and automatic centering | Working | The guarded teleop uses the physically verified `-0.40` installed center correction; steering firmware remains at zero additional trim. |
| Direction-change guard | PC and master guards installed | Drive and Sport are treated as the same forward direction; either forward mode still requires the full Neutral/brake guard before Reverse. |
| Sport mode | Software control added; vehicle test pending | `G` toggles the forward command between Drive and Sport while preserving the normal WASD, speed-level, ramp, braking, and E-stop behavior. Confirm the installed mode-line hardware before a powered Sport test. |
| Electric brake actuator | Bench movement and calibration working | IBT-2 direction, actuator movement, and potentiometer feedback were verified. Normal brake firmware uses the installed `945` released and `755` applied targets and has been flashed; full loaded stopping validation remains pending. |

The original controller remains available at `teleop/keyboard_teleop.py`. The latest tested game-style controller is `teleop/keyboard_teleop_wasd_guarded.py`.

## Safety

- Raise and securely support the drive wheels for bench testing.
- Keep a physical power disconnect and the manual brake within reach.
- Press `Esc` to send an E-stop, zero throttle, command Neutral, and request full brake before the program exits.
- The master and teleop both start in E-stop. Teleop shutdown repeats the E-stop command before closing; loss of the PC connection independently triggers the master's 500 ms watchdog E-stop.
- Software cannot apply the brake after its 12 V supply is removed. During vehicle shutdown, command E-stop and wait for brake application before disconnecting actuator power.
- Both direction guards check commanded/feedback gear state and elapsed time. They **do not measure actual vehicle speed**, so the operator must verify that the vehicle is physically stopped before changing direction.
- The electric brake has moved correctly during raised-vehicle commissioning, but loaded stopping and holding remain unverified. Keep the manual brake and physical power disconnect available.
- Do not connect a Teensy I/O pin directly to 12 V. Teensy pins are 3.3 V logic only.

## Repository Layout

```text
firmware/teensys/
  master.cpp       USB-to-CAN bridge
  throttle.cpp     MCP4725 throttle DAC and gear outputs
  steer.cpp        CL57T steering stepper control
  brake.cpp        Calibrated IBT-2 linear-actuator brake control
  brake_bench_test.cpp  USB-only, one-shot brake commissioning diagnostic
  dbw_can.h        Shared protected CAN protocol
  platformio.ini   PlatformIO environments for all four Teensys

teleop/
  keyboard_teleop_wasd_guarded.py  Latest tested game-style controller
  keyboard_teleop.py       Original controller/fallback
  teensy_serial.py         USB serial protocol helper
  throttle_test.py         Throttle-only diagnostic controller
```

## PC Setup

The tested PC environment is Windows PowerShell with Python, `pyserial`, and `pynput`.

```powershell
python -m pip install pyserial pynput
```

Find the master Teensy's COM port:

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```

Close Arduino Serial Monitor, PlatformIO Serial Monitor, and any other program using that port before starting teleop. A `Write timeout` or `could not open port` error usually means the wrong COM port was selected, the board disconnected, or another program still owns the port.

## Run The Tested Controller

From the repository root, replace `COM3` if the master appears on a different port:

```powershell
& "C:\Users\tapat\.platformio\penv\Scripts\python.exe" ".\teleop\keyboard_teleop_wasd_guarded.py" --port COM3
```

The full-path command used during testing was:

```powershell
& "C:\Users\tapat\.platformio\penv\Scripts\python.exe" "C:\Users\tapat\Documents\Codex\2026-09-29\pictures-of-prject-car-need-a\work\car2_ws\teleop\keyboard_teleop_wasd_guarded.py" --port COM3
```

The guarded teleop owns the physically verified `-0.40` installed center
correction, and steering firmware remains at zero additional trim. Use
`--steer-trim` only as a temporary measured override.

### Controls

| Key | Action |
| --- | --- |
| `1`, `2`, `3`, `4` | Select throttle level: 32%, 40%, 55%, or 75%. |
| Hold `W` | Drive forward at the selected level. |
| Hold `S` | Drive in reverse at the selected level. |
| Hold `A` / `D` | Steer left / right; release to return toward center. |
| `C` | Snap the steering command to center. |
| `Q` | Toggle between Neutral and active driving. |
| `G` | Toggle the forward mode between Drive and Sport. `W` uses the selected forward mode; `S` remains Reverse. |
| Hold `Space` | Command zero throttle and full electric brake. Loaded stopping performance is not yet verified. |
| `E` | Toggle E-stop. The controller starts in E-stop and returns to Neutral when E-stop is cleared. |
| `Esc` | Safe stop and exit. |

On startup the controller is in E-stop. Press `E` to clear E-stop, then press `Q`
to enable active driving, select a speed with `1` through `4`, and hold `W` or
`S`. Clearing E-stop never enables driving by itself.

Sport selection does not bypass any existing control or safety behavior. When
Sport is enabled, `W` requests `S` gear and keeps the same selected speed level,
high-speed ramp, steering, braking, E-stop, and direction-change guard. Pressing
`G` while moving between Drive and Sport first reduces commanded throttle to zero
until telemetry confirms the newly selected mode. Drive and Sport are both
forward modes, so changing between them does not invoke the forward/reverse
guard. Either one still requires the full Neutral/brake guard before Reverse.

### High-Speed Ramp

Speed levels 3 and 4 begin at level 2 and then rise using a monotonic-clock
slew rate. The default rate is `0.10` normalized throttle per second. With the
default levels `0.32,0.40,0.55,0.75`, level 3 takes 1.5 seconds and level 4
takes 3.5 seconds to rise from level 2:

```text
ramp time = (selected level - level 2) / ramp rate
```

The ramp resets whenever throttle is released, braking or E-stop is commanded,
gear feedback is not ready, Neutral is selected, or a direction-change guard is
active. A lower selected speed takes effect immediately. The rate can be changed
for a controlled test:

```powershell
python .\teleop\keyboard_teleop_wasd_guarded.py --port COM3 --high-speed-ramp-rate 0.10
```

This controls command timing only. The Votol controller has its own acceleration
rate-of-rise setting, and the vehicle has no wheel-speed feedback in this teleop.
Confirm the final rate with measured acceleration/current under the vehicle's
real load before ground operation.

### Direction Changes

Changing directly between forward and reverse starts the safety guard:

1. Throttle is set to zero.
2. Neutral and full brake are commanded.
3. Release both `W` and `S`.
4. The controller waits for Neutral feedback and the configured guard interval.
5. After the vehicle is physically stopped, press the desired direction again.

The PC guard's default minimum interval is 1.0 second. It can be changed for controlled testing, but the program will not allow less than 0.5 second. The master firmware independently enforces a fixed 1.0-second guard:

```powershell
python .\teleop\keyboard_teleop_wasd_guarded.py --port COM3 --direction-change-brake 0.50
```

### Adjust Speed Levels

The current tested defaults are `0.32,0.40,0.55,0.75`. Supply four strictly increasing normalized values to test a different set without editing the source. Values above `0.75` will still be capped by both the master and throttle firmware:

```powershell
python .\teleop\keyboard_teleop_wasd_guarded.py --port COM3 --speed-levels 0.34,0.42,0.58,0.78
```

## Firmware Build And Upload

The PlatformIO project defines one environment per Teensy: `master`, `throttle`, `steer`, and `brake`, plus a temporary `brake_bench` diagnostic. Connect only the intended board over USB when uploading, then press its Teensy program button if Teensy Loader requests it.

For the complete embedded-control update, flash these four vehicle firmware environments:

```powershell
cd .\firmware\teensys
pio run -e master -t upload
pio run -e throttle -t upload
pio run -e steer -t upload
pio run -e brake -t upload
```

Do **not** leave `brake_bench` installed for vehicle operation. It is a manual,
USB-only commissioning diagnostic that intentionally bypasses CAN and normal
closed-loop brake behavior.

Firmware responsibilities after this update:

- `master`: startup E-stop, 75% throttle ceiling, fresh gear-feedback requirement before torque, and an embedded forward-to-Reverse guard.
- `throttle`: independent 75% command ceiling in addition to the master's cap.
- `steer`: zero firmware correction; the measured `-0.40` center is applied by teleop.
- `brake`: calibrated closed-loop IBT-2 control using A0 position feedback and 20 kHz PWM.
- `teleop`: startup/shutdown E-stop, keyboard mapping, selectable speed presets, level 3/4 ramping, steering correction, and the operator-facing direction guard.

## Hardware Notes

- Master: USB serial at 115200 baud; CAN1 at 250 kbit/s.
- CAN wiring: yellow/green twisted pair on this vehicle. Confirm polarity and termination before relying on color alone.
- Throttle: Votol EM70 controller commanded by an MCP4725 12-bit DAC. Firmware probes I2C addresses `0x62` and `0x60`; idle output is 0.80 V and maximum is 3.30 V.
- Steering: 23HS30-5004D-E1000 closed-loop stepper with a CL57T driver.
- Underglow: 12 V power comes from the fused 12 V bus. Master Teensy pins 2-5 are 3.3 V logic-control signals only.
- Brake: TS-LD-HS, 12 V, 50 mm stroke, 750 N, 10 mm/s linear actuator with nominal 10 kOhm position feedback, driven through an IBT-2.
- Brake Teensy firmware assignments: pin 19 = retract/release, pin 18 = extend/apply, and A0/pin 14 = potentiometer feedback.
- Installed brake calibration: raw `945` = released; raw `745` = measured mechanical apply limit; normal full-brake target `755` preserves a 10-count margin.
- IBT-2 logic enable lines must be asserted, logic ground must be common with the Teensy, and actuator motor power must come from the fused 12 V system rather than a Teensy pin.
- Brake PWM runs at 20 kHz to move the IBT-2 switching tone above the normal audible range. The BTS7960 device is rated for PWM operation up to 25 kHz.

Wire colors are not treated as authoritative. The actuator conductors were observed as black, red, yellow, white, and teal/green, but the motor pair, potentiometer endpoints, and wiper must be identified electrically before further powered testing.

## Telemetry Quick Reference

The live controller display includes master telemetry:

| Field | Meaning |
| --- | --- |
| `w` | Master serial watchdog active. |
| `t` | Master throttle command. |
| `tv` | Throttle-node DAC voltage feedback. |
| `tm` | Throttle-node gear code (`0=N`, `1=D`, `2=S`, `3=R`). |
| `sp` | Steering position percent. |
| `sf` | Steering status flags. |
| `bl` | Brake actuator length in inches. |
| `bf` | Brake status flags. |

## Test And Change Log

### 2026-10-06 21:42 PDT Release Verification

- Confirmed the tested `-0.40` steering correction remains in teleop while steering firmware stays at the original `0.00` center, preventing stacked offsets.
- Rebuilt all five PlatformIO environments: `master`, `throttle`, `steer`, `brake`, and the temporary `brake_bench` diagnostic.
- Recompiled all Python teleop tools and reran deterministic startup E-stop, E/Q gating, level 3/4 ramp, ramp-reset, Sport/Reverse guard, and shutdown-state checks.
- Confirmed shutdown sends five complete E-stop packets with Neutral, zero throttle, full brake, and the installed steering correction.

### 2026-10-06 Brake Safety Update

- Corrected the measured actuator direction: pin 19 retracts/releases and pin 18 extends/applies.
- Calibrated installed potentiometer feedback to raw `945` released and raw `745` at the mechanical apply limit; normal control targets `755` for a 10-count margin.
- Added a USB-only brake commissioning firmware with short/long, one-shot, automatically stopped pulses. The diagnostic was used to verify both motion and changing potentiometer feedback.
- Changed the master power-up state to E-stop, Neutral, zero throttle, and full brake.
- Strengthened teleop shutdown by sending the complete E-stop state five times before closing. Abrupt PC or USB loss remains covered by the master's 500 ms watchdog.
- Changed brake PWM to 20 kHz to suppress the audible IBT-2/motor switching tone while staying below the BTS7960's specified 25 kHz limit.
- Vehicle shutdown must still apply E-stop before removing actuator 12 V; software cannot move an unpowered brake actuator.

### 2026-10-06 Steering Center Reset

- Removed the permanent `+0.10` steering-firmware correction and restored the
  original `0.00` center.
- The guarded teleop defaults to the physically verified `-0.40`; normal
  commands should omit `--steer-trim`.
- Temporarily widened the command-line steering-trim calibration range to
  `-0.50..+0.50`; the final steering command remains clamped to `-1.00..+1.00`.
- Raised-vehicle testing found the installed center at `-0.40`. The firmware
  correction did not reproduce the tested movement after flashing, so the
  correction remains in the verified teleop command path and firmware is reset
  to zero to prevent stacked offsets.

### 2026-10-06 14:32 PDT

- Added `G` as a Drive/Sport toggle in the guarded WASD controller. Sport keeps
  the same `W`/`S`/`A`/`D`, speed-level, ramp, brake, and E-stop behavior as the
  default mode.
- Updated both PC and embedded direction guards so Drive and Sport are treated
  as the same forward direction, while either forward mode to/from Reverse still
  requires the full Neutral/brake guard.
- Kept the current throttle-node mode-output implementation unchanged. The
  photographed reference uses specific analog B/C/D mode voltages, so the
  installed mode-generation hardware must be confirmed before powered Sport use.

### 2026-10-05 23:07 PDT

- Changed the guarded WASD controller to start in E-stop; clearing E-stop leaves the controller in Neutral and requires a separate `Q` press to enable driving.
- Added a monotonic-clock throttle ramp for speed levels 3 and 4. Both begin at level 2 and rise at the default `0.10` normalized throttle per second.
- At the current levels, level 3 reaches its target after 1.5 seconds and level 4 after 3.5 seconds. Release, braking, E-stop, missing gear feedback, Neutral, and direction guarding reset the ramp.
- Added the `--high-speed-ramp-rate` runtime option for measured tuning without source edits.
- Passed Python compilation and deterministic startup, E-stop, ramp-duration, release-reset, brake, and stop-state checks. Raised-vehicle and loaded ground tests remain pending.

### 2026-10-05 21:48 PDT

- Restored the known-working `342d75d` snapshot in an isolated checkout without overwriting the newer repository work.
- Successfully built and reflashed the master and throttle Teensys from that snapshot; steering remained functional and was not reflashed during recovery.
- Confirmed that the persistent acceleration failure was caused by a wiring fault rather than the WASD key handling, PowerShell command, master serial link, or GitHub source history.
- Corrected the wiring fault and returned to the tested game-style WASD controls.
- Removed the temporary no-feedback diagnostic mode before publication; it was never committed or pushed.
- Verified that GitHub already contained the master, throttle, steering, and guarded-teleop updates. This README was the remaining unpublished tracked file.
- The vehicle is currently using the recovery master/throttle firmware. Reflash and bench-test the newer embedded master, throttle, and steering policies together before treating them as the installed vehicle configuration.

### 2026-10-05 19:13 PDT

- Moved the tested `+0.10` center correction from the latest teleop into steering firmware; teleop now defaults to zero additional trim.
- Added a 75% throttle ceiling to both master and throttle firmware.
- Added a master-side direction guard requiring Neutral, full brake, fresh gear feedback, a one-second dwell, and released throttle before an opposite gear can apply torque.
- Kept the PC guard and taught the master to recognize its completed Neutral/brake dwell so the full delays do not intentionally stack.
- Successfully compiled PlatformIO environments `master`, `throttle`, and `steer` for Teensy 4.1.
- Vehicle testing is pending; no Teensys were flashed during this documentation/build step.

### 2026-10-05 18:47 PDT

- Made the tested `+0.10` steering-center correction the latest controller's built-in default.
- Retained `--steer-trim` as an optional runtime override for future calibration.

### 2026-10-05 18:44 PDT

- Renamed the latest controller from `keyboard_teleop_wasd.py` to `keyboard_teleop_wasd_guarded.py` so its game-style controls and direction-change protection are clear.
- Left the original `keyboard_teleop.py` unchanged as a fallback.

### 2026-10-05 18:37 PDT

- Added and locally tested `teleop/keyboard_teleop_wasd_guarded.py` while the car was securely supported off the ground.
- Verified steering, automatic centering, forward throttle, reverse throttle, four selectable speed levels, Neutral toggle, and guarded direction changes.
- Corrected the small left steering offset with `--steer-trim 0.10`.
- Confirmed that the original teleop still works and remains available as a fallback.
- Verified Python syntax and controller state transitions locally.
- Paused electric brake work after CAN commands and brake telemetry were observed but the actuator did not move.
- No brake calibration values or actuator wire-color assignments should be considered final.

## Known Follow-Up Work

1. Validate electric-brake stopping and holding performance under controlled vehicle load with a manual brake and physical power disconnect available.
2. Confirm the installed Sport mode-line hardware before conducting a powered Sport-mode test.
3. Reconfirm that steering center remains correct with no `--steer-trim` argument after the complete firmware set is installed.
4. Validate the embedded forward/Reverse guard and startup/shutdown E-stop behavior with the drive wheels securely raised.
5. Tune the four throttle levels and high-speed ramp rate during controlled ground testing while keeping the 75% firmware ceiling in place.

