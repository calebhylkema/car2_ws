# 2026-10-20 — Vehicle Control Handoff and Steering Tuning

This dated folder contains the four-node Teensy 4.1 vehicle-control baseline used on this car, the host controller, diagnostic firmware, tests, recorded steering limits, and reproducible Windows commands. This handoff now includes the 2026-10-30 source updates. **Run the tools in this folder**, rather than similarly named older scripts at the repository root. Nothing is flashed or moved by cloning, setup, verification, building, or offline tests.

The operator reported working acceleration and steering before this handoff. Steering currently uses manually referenced step counts with selected travel limits **−8,250 left / +10,000 right**. Its analog sensor still needs validation; automatic saved-center calibration and startup centering are **not enabled**. Remaining work is **tuning and brake**; [earlier steering tuning notes](STEERING-TUNING-NEXT.md) remain available as background. See [validation and limits of the evidence](VALIDATION.md).

## Current source — 2026-10-30

The operator approved the working baseline for publication on 2026-10-23. That baseline passed 37 host tests, 14 firmware/mock executables, master and throttle PlatformIO builds, and package hash verification. The 2026-10-30 host tuning slows steering return and initial throttle ramp and was approved for publication on 2026-10-30; all 37 host tests pass, with vehicle testing pending. Restart teleop; these two changes require no firmware flash. Hardware flashing is operator-managed. Remaining work is **tuning and brake**.

- Missing node feedback no longer triggers the master/teleop all-node CAN lock; feedback remains visible. Steering reference, E-stop, command watchdogs, and direction guards remain in place.
- D ↔ R Neutral/brake dwell is 0.5 seconds in teleop and master. Release the old direction key, then hold the new one; it continues after the pause without a second press. Neutral confirmation remains required, W+S together inhibit driving, and throttle waits for the newly selected gear. Throttle-node gear-settling times are unchanged, so complete changeover may take longer than the dwell. The timer does not measure vehicle speed; change direction only while physically stopped.
- Keys 1–4 select 15%, 30%, 55%, and 85% throttle command levels. Every level ramps from zero at 10 percentage points/s up to 30%, then 7.5 points/s above 30%. From zero, levels 1–4 take about 1.5, 3, 6.3, and 10.3 seconds. Changing from a settled level 2 to 3 takes about 3.3 seconds; level 3 to 4 takes 4 seconds. Level changes continue from the current command; lower targets and throttle release/brake/E-stop cut demand promptly. Master and throttle firmware both cap demand at 85%; flash both to enable the new maximum.
- Steering uses a 24,000 pulses/s ceiling, 51,200 pulses/s² acceleration, and a 25 µs step pulse. A valid single end-switch hit returns steering to the recorded Z position; invalid switch states and command timeouts still clear reference.
- Normal A/D steering commands build from center to full travel over 1 second and return from full travel to Z over 2 seconds after release. Reversing A/D passes smoothly through center. This uses elapsed time in teleop; motor speed, travel limits, and firmware end-switch recovery are unchanged. These are command timings, not measured wheel travel times. Restart teleop to use the change.
- Every teleop startup requires physically straight wheels and a fresh Z confirmation, even when the Teensy retains an earlier reference. The alignment reminder stays at the beginning of the status line until Z is selected and steering reports a reference; then E and Q can enable driving.
- Brake direction pins are corrected: extend/apply GPIO19, retract/release GPIO18. Brake tuning remains unfinished.

Build/upload the current source in `car2_ws/firmware/teensys` with PlatformIO. Packaged HEX files, frozen evidence, and `source-provenance.json` preserve the earlier baseline and do not represent these source updates. Board serial/COM assignments below are historical: boards have been moved and reflashed. Confirm the physical role and current port before uploading or running control.

## Get the package onto a new Windows PC

Install Git and desktop Python 3.11 (including the Python launcher). Use a PowerShell console with keyboard focus; close other serial monitors before opening a Teensy port. Internet access is needed for first-time dependency/toolchain installation. The tools resolve paths relative to this folder and do not depend on the original developer's account or workspace.

```powershell
git clone https://github.com/calebhylkema/car2_ws.git
Set-Location .\car2_ws\handoffs\2026-10-20-vehicle-control-and-steering-tuning
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Verify-Package.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Setup.ps1 -Build
```

If already cloned, run `git pull --ff-only` from a clean checkout, then enter this dated folder. Setup creates `.venv`, installs pinned Python packages, and builds the four operational environments. It also installs PlatformIO's Teensy USB uploader. If `py` is unavailable, pass `-Python` with a Python executable path to Setup. On Windows, install the Teensy USB driver through PJRC's Teensy installer if Device Manager does not recognize the board. No compiler or uploader executable is committed here. Native C++ tests additionally need a working MinGW `g++` or equivalent host C++11 compiler.

## “get com” — find ports and identities

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
& .\.venv\Scripts\python.exe .\outputs\car2-four-node-can-test\can_test_usb.py list
```

The first command only lists ports. The second includes the USB serial used by the launchers. COM numbers can change; they are not node identities.

| Physical slot | Known application USB serial | Last usual COM | Operational image |
|---|---|---|---|
| Master | 18637540 | COM3 | `outputs/car2-vehicle-firmware-r02/master.hex` |
| Steering | 20369970 | COM5 | `outputs/car2-steering-calibration/steering-operational-limits-r01.hex` |
| Acceleration/throttle | 20543040 | COM9 | `outputs/car2-vehicle-firmware-r02/throttle.hex` |
| Brake | 20542540 | COM10 | `outputs/car2-vehicle-firmware-r02/brake.hex` |

## Archived baseline installation, one board at a time

Switch off traction, steering-driver, and brake-actuator power; keep only the selected Teensy connected by USB. Close teleop/serial monitors. The explicit flag records that actuator power is off; software cannot measure the power state. The packaged HEX files preserve the recorded baseline. Firmware builds are checked separately and do not replace these archived images.

Run the appropriate single command, moving USB to the physical slot before each invocation:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Flash-Node.ps1 -Role master -ActuatorPowerOff
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Flash-Node.ps1 -Role steering -ActuatorPowerOff
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Flash-Node.ps1 -Role throttle -ActuatorPowerOff
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Flash-Node.ps1 -Role brake -ActuatorPowerOff
```

The uploader checks hashes and the unique application/bootloader identity, writes only that board, waits for USB to return, and records the result under `runtime/installations`. It sends no movement command. If the application cannot enter its bootloader, keep actuator power off, press/release that board's Program button once, and rerun. A successful upload/application return is not independent runtime firmware readback.

`-ValidateOnly` checks the package and installed upload tools without opening USB. For a replacement board, first identify its serial and physical slot, then supply `-Serial <actualSerial>` explicitly. The host scripts intentionally retain this car's known serials: make a **new dated copy** of the handoff and update its identity checks for replacement hardware before running control/diagnostics. Do not bypass an identity mismatch on a guessed COM number.

## Run full-car keyboard control through master

Power the installed operational nodes and their CAN modules. Missing feedback is displayed but does not itself lock control. See [wiring](WIRING.md), including VIN/USB power isolation. For first powered tests, keep drive wheels securely raised, steering unloaded, people clear, and a physical power cutoff available. Center the front wheels physically before referencing. USB stays on master; other boards can be powered by the vehicle.

```powershell
$masterPort = Read-Host "Enter confirmed master port (e.g. COM5)"
if ($masterPort -match '^\d+$') { $masterPort = "COM$masterPort" }
& .\.venv\Scripts\python.exe .\car2_ws\teleop\keyboard_teleop_wasd_guarded.py --port $masterPort
```

Use the confirmed current master port. The older `Run-Teleop.ps1` launcher assumes a historical master serial and may reject the current board assignment. Teleop starts in E-stop. **Z does not move the wheels:** only press it while physically straight/stationary to declare the current position zero.

| Key | Action |
|---|---|
| Z, while E-stop is active | Declare physically straight current position as center |
| F2, while E-stop is active | Toggle manual Z reference availability |
| E | Toggle E-stop; clearing it requires current master telemetry and a valid steering reference |
| Q | Toggle drive enable after E-stop is cleared |
| W / S, or up/down arrows | Forward / reverse; direction changes use neutral/brake dwell |
| A / D, or left/right arrows | Steer left / right |
| C | Command the referenced steering center |
| 1 / 2 / 3 / 4 | Select 15% / 30% / 55% / 85% throttle command |
| G | Toggle sport gear selection |
| Space | Brake |
| Esc | Exit with software E-stop |

Start with level **1**. A released key does not establish a new center. `-DisableManualReference` starts with Z disabled; F2 can toggle it in E-stop. This is manual-reference control, not automatic sensor centering. A valid single end-switch hit returns to Z. A steering reset, command timeout, or invalid switch state can invalidate reference; physically straighten and reference again only after resolving the cause.

If the console appears “locked,” read its state: The `tr/sr/br` fields report node communication without the former `CHECK-CAN` lock. `NEED-REF (Z)` requires physical centering then Z; `Z-OFF (F2)` requires enabling manual reference if appropriate. `NO-MASTER` means stale USB telemetry. Stop any PowerShell text selection (Esc), focus the console, and check identities/power/firmware/CAN rather than removing interlocks. A CAN-only diagnostic image is incompatible with normal full-car teleop.

## Reproduce communication and steering tests

The CAN-only test replaces firmware on all four boards. It contains no actuator control. Keep actuator power off, connect one USB at a time when prompted, and power the CAN logic/transceivers in their actual slots. The script stores role assignments in `runtime/can` and finishes by reading master:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\outputs\car2-four-node-can-test\Flash-CAN-Test.ps1 -ActuatorPowerOff
# Subsequent monitor-only session, after the diagnostic images are installed:
powershell -NoProfile -ExecutionPolicy Bypass -File .\outputs\car2-four-node-can-test\Flash-CAN-Test.ps1 -MonitorOnly
```

Expected: THROTTLE/STEERING/BRAKE=OK, increasing replies, and no reported CAN errors. Restore **all four operational images** afterward using Flash-Node above. CAN-test pass proves the diagnostic command/reply path; it does not prove actuator movement or normal firmware compatibility by itself.

A passive **master-only** normal CAN check (no control commands) is also available:

```powershell
& .\.venv\Scripts\python.exe .\car2_ws\teleop\check_can.py --port COM3
```

Passive normal steering status, with USB moved to steering:

```powershell
& .\.venv\Scripts\python.exe .\work\read_steering_status.py --port COM5
```

Finite normal master-to-steering right → center → left → center test, with all operational nodes powered, wheels unloaded, steering physically straight, master USB connected, and cutoff ready:

```powershell
& .\.venv\Scripts\python.exe .\outputs\car2-steering-calibration\test_master_steering_limits.py --center-confirmed
```

That test **moves steering to both saved soft boundaries**; throttle stays zero, gear N, brake commanded applied. It logs software position and feedback under `runtime/master`; an operator must separately confirm actual movement. It is not a braking-force test. For direct throttle/DAC and analog-steering diagnostics, use [DIAGNOSTICS.md](DIAGNOSTICS.md); restore operational firmware after each temporary image.

## Build and test without the car

```powershell
# Four operational firmware builds; no upload:
& .\.venv\Scripts\python.exe -m platformio run -d .\car2_ws\firmware\teensys
# Host Python tests plus fourteen C++ firmware/mock executables:
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Test-Package.ps1 -Compiler 'C:\path\to\g++.exe' -BuildFirmware
```

Individual build environments are in the **nested** `car2_ws/firmware/teensys/platformio.ini`. `-e steer` builds normal steering; `-e steer_sensor_test` builds the temporary analog diagnostic. Build outputs are in its `.pio/build/<env>/firmware.hex`. To install a newly changed build, with only the selected board attached and actuator power off, use PlatformIO `run -d ... -e <env> -t upload`; this does not use the archived-image identity/hash guard. Verify slot, environment, and runtime output first. The dated baseline uploader always installs archived images, never implicitly a recent `.pio` build.

Frozen evidence is under `evidence/`. New logs are under ignored `runtime/`. `manifest.json` hashes every packaged file except itself; `source-provenance.json` records the source comparison. `history/local-readme-before-this-handoff.md` is historical and contains earlier, superseded states. An earlier editable SVG/PDF schematic is preserved under `history/schematic-r01`; see its historical-status note before using it. The current-version notes above supersede the original snapshot instructions; older root documentation describes earlier code.
