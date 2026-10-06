# car2_ws

Teensy 4.1 firmware and a Windows keyboard controller for the `car2` drive-by-wire test vehicle.

## Current Status

Last updated: **2026-10-05 21:48 PDT (UTC-07:00)**

| System | Status | Notes |
| --- | --- | --- |
| Master Teensy and CAN link | Working on recovery firmware | USB serial at 115200 baud and vehicle CAN at 250 kbit/s were restored using the known-working `342d75d` snapshot. |
| Throttle and gear control | Working after wiring repair | A wiring fault caused the apparent software regression. The fault was corrected after restoring the known-working master and throttle firmware. The newer 75% firmware ceilings remain ready for a separate controlled vehicle test. |
| Steering and automatic centering | Working | Steering remained operational throughout recovery. The current recovery setup uses the tested `+0.10` teleop correction; the repository also contains the firmware-owned correction for the next coordinated steering flash. |
| Direction-change guard | PC guard working | The tested PC guard is in the updated WASD controller. The newer master-side guard is committed and compiled but is not currently installed after the recovery rollback. |
| Electric brake actuator | Not working yet | CAN brake commands are visible, but the actuator has not moved. Hardware, feedback wiring, calibration, and IBT-2 behavior still require diagnosis. |

The original controller remains available at `teleop/keyboard_teleop.py`. The latest tested game-style controller is `teleop/keyboard_teleop_wasd_guarded.py`.

## Safety

- Raise and securely support the drive wheels for bench testing.
- Keep a physical power disconnect and the manual brake within reach.
- Press `Esc` to send an E-stop, zero throttle, command Neutral, and request full brake before the program exits.
- Both direction guards check commanded/feedback gear state and elapsed time. They **do not measure actual vehicle speed**, so the operator must verify that the vehicle is physically stopped before changing direction.
- The electric brake is currently unverified. Do not depend on it to stop or hold the vehicle.
- Do not connect a Teensy I/O pin directly to 12 V. Teensy pins are 3.3 V logic only.

## Repository Layout

```text
firmware/teensys/
  master.cpp       USB-to-CAN bridge
  throttle.cpp     MCP4725 throttle DAC and gear outputs
  steer.cpp        CL57T steering stepper control
  brake.cpp        IBT-2 linear-actuator brake control (unresolved)
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

The repository's latest steering firmware owns the tested `+0.10` correction,
so the latest teleop defaults to zero additional trim. While using the recovery
firmware that was restored during the 2026-10-05 wiring diagnosis, run the
known-working `342d75d` teleop snapshot or pass `--steer-trim 0.10` explicitly.

### Controls

| Key | Action |
| --- | --- |
| `1`, `2`, `3`, `4` | Select throttle level: 32%, 40%, 55%, or 75%. |
| Hold `W` | Drive forward at the selected level. |
| Hold `S` | Drive in reverse at the selected level. |
| Hold `A` / `D` | Steer left / right; release to return toward center. |
| `C` | Snap the steering command to center. |
| `Q` | Toggle between Neutral and active driving. |
| Hold `Space` | Command zero throttle and full electric brake. The electric brake hardware is not yet verified. |
| `E` | Toggle E-stop. |
| `Esc` | Safe stop and exit. |

On startup the controller is in Neutral. Press `Q` once to enable active driving, select a speed with `1` through `4`, and then hold `W` or `S`.

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

The PlatformIO project defines one environment per Teensy: `master`, `throttle`, `steer`, and `brake`. Connect only the intended board over USB when uploading, then press its Teensy program button if Teensy Loader requests it.

For the embedded-control update, flash exactly these three nodes:

```powershell
cd .\firmware\teensys
pio run -e master -t upload
pio run -e throttle -t upload
pio run -e steer -t upload
```

Do **not** flash the brake Teensy for this update. Its unresolved actuator work is unchanged.

Firmware responsibilities after this update:

- `master`: 75% throttle ceiling, fresh gear-feedback requirement before torque, and embedded D-to-R/R-to-D guard.
- `throttle`: independent 75% command ceiling in addition to the master's cap.
- `steer`: permanent `+0.10` normalized center correction.
- `teleop`: keyboard mapping, selectable speed presets, and the existing operator-facing guard.

## Hardware Notes

- Master: USB serial at 115200 baud; CAN1 at 250 kbit/s.
- CAN wiring: yellow/green twisted pair on this vehicle. Confirm polarity and termination before relying on color alone.
- Throttle: Votol EM70 controller commanded by an MCP4725 12-bit DAC. Firmware probes I2C addresses `0x62` and `0x60`; idle output is 0.80 V and maximum is 3.30 V.
- Steering: 23HS30-5004D-E1000 closed-loop stepper with a CL57T driver.
- Underglow: 12 V power comes from the fused 12 V bus. Master Teensy pins 2-5 are 3.3 V logic-control signals only.
- Brake: TS-LD-HS, 12 V, 50 mm stroke, 750 N, 10 mm/s linear actuator with nominal 10 kOhm position feedback, driven through an IBT-2.
- Brake Teensy firmware assignments: pin 19 = RPWM/apply, pin 18 = LPWM/release, and A0/pin 14 = position feedback.
- IBT-2 logic enable lines must be asserted, logic ground must be common with the Teensy, and actuator motor power must come from the fused 12 V system rather than a Teensy pin.

Wire colors are not treated as authoritative. The actuator conductors were observed as black, red, yellow, white, and teal/green, but the motor pair, potentiometer endpoints, and wiper must be identified electrically before further powered testing.

## Telemetry Quick Reference

The live controller display includes master telemetry:

| Field | Meaning |
| --- | --- |
| `w` | Master serial watchdog active. |
| `t` | Master throttle command. |
| `tv` | Throttle-node DAC voltage feedback. |
| `tm` | Throttle-node gear code (`0=N`, `1=D`, `3=R`). |
| `sp` | Steering position percent. |
| `sf` | Steering status flags. |
| `bl` | Brake actuator length in inches. |
| `bf` | Brake status flags. |

## Test And Change Log

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

1. Identify the actuator's two motor conductors, two potentiometer endpoints, and wiper using resistance measurements with power removed.
2. Verify IBT-2 output under actuator load and confirm that the actuator itself moves from a separately fused 12 V source before changing control logic.
3. Measure the true released/applied A0 values and replace the provisional `ADC_RELEASED=300` and `ADC_APPLIED=430` values in `brake.cpp`.
4. Test the electric brake at low force/current with a physical stop method available.
5. Flash and bench-test the master, throttle, and steering nodes with the drive wheels securely raised.
6. Confirm that center remains correct with no `--steer-trim` argument and that opposite-direction requests cannot bypass the embedded guard.
7. Tune the four throttle levels during a controlled ground test while keeping the 75% firmware ceiling in place.

