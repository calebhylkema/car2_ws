# 2026-10-20 — Temporary diagnostics and restore procedures

These commands are run from this dated folder after Setup -Build. Installing a diagnostic replaces that node's operational firmware. Switch actuator power off for every firmware change, use only the selected board on USB, and restore the normal image before returning to full-car control. For powered movement: raised/unloaded wheels, people clear, cutoff ready, operator observation throughout. Do not use the motor step position as proof of physical motion.

## Analog steering retest

Keep driver power off and install on steering USB serial20369970:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Flash-Node.ps1 -Role steering -Image sensor -ActuatorPowerOff
& .\.venv\Scripts\python.exe .\outputs\car2-steering-calibration\steering_sensor_test.py --port COM5
```

The default host operation sends STOP/P and captures stationary status; it does not jog. Firmware marker must be STEER_SENSOR_TEST_V1. Resolve electrical/sensor issues before relying on it for calibration. A0 is sampled at 12 bits; operational raw debug readings may use a different ADC resolution, so do not compare raw numbers without checking mode.

With wheels physically straight/stationary, power conditions suitable for a finite unloaded test, and no fault:

```powershell
& .\.venv\Scripts\python.exe .\outputs\car2-steering-calibration\steering_sensor_test.py --port COM5 --zero --center-confirmed
# Positive is right; this commands 250 motor pulses, not 250 degrees:
& .\.venv\Scripts\python.exe .\outputs\car2-steering-calibration\steering_sensor_test.py --port COM5 --steps 250 --expected-position 0
# Return only if actual status still reports position250 and the linkage is clear:
& .\.venv\Scripts\python.exe .\outputs\car2-steering-calibration\steering_sensor_test.py --port COM5 --steps -250 --expected-position 250
```

Host and firmware restrict travel to −8250/+10000 around the explicitly declared zero. Each burst is bounded to5000 pulses and has keepalive/timeouts. Abort on faults or unexpected travel; do not retry a guessed position after reset. No CAN control, EEPROM profile, or automatic sensor-based motion is provided by this diagnostic. Stop and turn driver power off before restoring:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Flash-Node.ps1 -Role steering -ActuatorPowerOff
```

The older `Flash-Steering-Limit-Test.ps1` / `steering_limit_test.py` and V3 HEX are retained to reproduce the recorded boundary-selection session. V3 has no encoder position control and does not enforce the current center-referenced software bounds in firmware; do not replay its old relative positions after a reset. Prefer the bounded sensor diagnostic for the next work. In this handoff, both `-RestoreOperational` and `-InstallOperationalLimits` restore the newer mapped operational image, not the superseded steering image.

## Direct throttle/DAC test

Switch traction power off, select throttle USB serial20543040, and install:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Flash-Node.ps1 -Role throttle -Image throttle-usb -ActuatorPowerOff
& .\.venv\Scripts\python.exe .\work\throttle_usb_test.py --port COM9
```

Default operation probes DAC/I2C and neutral state. It cannot prove voltage at the VOTOL or motor readiness. For a finite forward test only after traction power is on and drive wheels are securely raised/everyone clear:

```powershell
& .\.venv\Scripts\python.exe .\work\throttle_usb_test.py --port COM9 --run --wheels-raised-confirmed --vehicle-power-on-confirmed
```

The test starts/stops at idle/neutral, requires DAC readback, sends keepalive, and finishes a finite forward sequence rather than offering unrestricted acceleration. Read its log under `runtime/throttle` and observe the wheels separately. Switch traction power off and restore:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\Flash-Node.ps1 -Role throttle -ActuatorPowerOff
```

The firmware sources also retain CAN pair, USB steering jog, and brake bench diagnostics. They are isolated commissioning tools, not substitutes for matched normal firmware. The bench brake source is not part of default builds; check direction, feedback, mechanical travel and power isolation before using it. Normal brake software feedback/command status is not proof of loaded stopping performance.

## Brake hold diagnostic — 2026-11-02

The temporary PlatformIO `brake_bench` environment now builds `BRAKE_USB_HOLD_TEST_V1`, not the earlier timed-pulse test. Normal driving uses the `brake` environment.

After installing `brake_bench` on the physically identified brake Teensy, close serial monitors and run from this handoff folder, replacing COM3 with the confirmed current brake port:

```powershell
& .\.venv\Scripts\python.exe .\car2_ws\teleop\brake_usb_hold_test.py --port COM3
```

Hold A or Space to extend/apply (GPIO18); hold R to retract/release (GPIO19). Releasing the key stops motion. X stops and Esc exits. Conflicting apply/release keys stop motion. The helper sends held-key updates; a serial monitor alone cannot provide key-release events. Firmware starts stopped, uses continuous full-duty drive, stops at measured raw74/raw564 travel limits, and stops after 150 ms without commands. The endpoint latch prevents repeated drive into a reached limit while the same direction remains held.

This diagnostic ignores CAN and is not normal vehicle firmware. Restore the current `brake` environment for car operation: it uses raw74 released and **provisional raw277 full braking**, not the diagnostic raw564 travel limit. Normal control retains existing E-stop/watchdog brake application. The physical brake test values do not establish loaded stopping force.
