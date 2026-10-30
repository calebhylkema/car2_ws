# car2_ws

Teensy 4.1 firmware and Windows keyboard control for the `car2` project car.

## Current source — 2026-10-30

Use [the current source, setup, and operating instructions](handoffs/2026-10-20-vehicle-control-and-steering-tuning/README.md).
The active project is inside:

```text
handoffs/2026-10-20-vehicle-control-and-steering-tuning/car2_ws/
```

Its `teleop/keyboard_teleop_wasd_guarded.py` is the current controller. Open its `firmware/teensys` folder in VS Code for PlatformIO. Root-level `teleop` and `firmware` files are older versions.

## Controls

| Key | Action |
| --- | --- |
| `1`, `2`, `3`, `4` | Select throttle command level: **15%, 30%, 55%, or 85%**. |
| Hold `W` / up arrow | Forward at the selected level, with ramp-up. |
| Hold `S` / down arrow | Reverse at the selected level, with ramp-up. |
| Hold `A` / left arrow or `D` / right arrow | Steer left/right. Center-to-full command takes **1 second**; releasing returns from full command to Z over **2 seconds**. |
| `Z`, while E-stop is active | Declare the physically straight wheels as center. Required at every teleop startup. Z records the position; it does not straighten the wheels. |
| `F2`, while E-stop is active | Toggle manual Z reference availability. |
| `C` | Set the steering command directly to the referenced center. |
| `Q` | Toggle between Neutral and active driving. |
| `G` | Toggle forward mode between Drive and Sport. Reverse remains Reverse. |
| Hold `Space` | Command zero throttle and full electric brake. Loaded stopping performance remains unverified. |
| `E` | Toggle E-stop. Clearing requires current master telemetry and a valid steering reference. |
| `Esc` | Send E-stop and exit. |

At startup, physically straighten the wheels, press **Z → E → Q**, then select a level and hold W or S. These instructions assume manual reference is enabled. Confirm the current master COM port; historical COM numbers and USB role assignments are not authoritative.

## Acceleration ramp

Every level starts from zero command when W/S is reapplied. The command rises at **10 percentage points per second up to 30%**, then **7.5 percentage points per second above 30%**.

| Level | Throttle command | Approximate ramp time from zero |
| --- | --- | --- |
| 1 | 15% | 1.5 seconds |
| 2 | 30% | 3 seconds |
| 3 | 55% | 6.3 seconds |
| 4 | 85% | 10.3 seconds |

Changing levels while holding the same direction continues from the current command. From settled level 2 to 3 takes about **3.3 seconds**; level 3 to 4 takes **4 seconds**. Selecting a lower level reduces the command promptly. Throttle release, braking, E-stop, or an inhibited drive state resets the ramp.

These are throttle-command percentages and timings, not measured motor power, torque, or vehicle speed. The 2026-10-30 ramp and steering-return updates passed all **37 host tests**; vehicle testing is pending. Restart teleop to use them; these two host changes require **no Teensy reflash**.

## Direction changes and CAN feedback

Release the old direction key, then hold the new one after the vehicle has stopped. Teleop and master use a **0.5-second minimum Neutral/brake dwell**. Held input continues after the pause without a second press; Neutral feedback and confirmation of the requested gear are still required. Throttle-node settling can add time. The guard does not measure vehicle speed. W and S held together inhibit driving.

Missing node feedback remains visible without the former all-node CAN lock. Steering-reference checks, E-stop, command watchdogs, and gear guards remain active.

## Firmware, wiring, and remaining work

The operational PlatformIO environments are **master, throttle, steer, and brake** in the active project's `firmware/teensys` folder. Other environments are diagnostics. Upload the environment matching the physically identified board. Packaged HEX files preserve an older baseline; build the current source when flashing current firmware.

- [Current setup, run, build, and test instructions](handoffs/2026-10-20-vehicle-control-and-steering-tuning/README.md)
- [Current wiring and pin assignments](handoffs/2026-10-20-vehicle-control-and-steering-tuning/WIRING.md)
- [Steering sensor validation notes](handoffs/2026-10-20-vehicle-control-and-steering-tuning/STEERING-TUNING-NEXT.md)
- [Validation and limits of recorded evidence](handoffs/2026-10-20-vehicle-control-and-steering-tuning/VALIDATION.md)

Remaining work includes **brake tuning/validation and steering encoder integration**. The current steering firmware reports the analog sensor reading but does not use it for position correction; normal steering uses manually referenced step counts.
