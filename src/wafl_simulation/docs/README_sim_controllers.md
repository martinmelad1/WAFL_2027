# `sim_controllers.yaml` — Gazebo joint controller tuning

File: `wafl_simulation/config/sim_controllers.yaml`. It is read by `urdf/wafl_gz_plugins.xacro` (through `xacro.load_yaml`) when the robot description is built. Change a value, rebuild (`colcon build --packages-select wafl_simulation`) and relaunch. **No regeneration is needed.**

`scripts/generate_assets.py` writes this file **only if it does not exist**, so your tuning is never overwritten.

## `wheel_drive` — front-left / front-right drive wheels

The command is a wheel angular velocity in **rad/s** (`/front_*_wheel_cmd_vel`).

| Variable | Default | Meaning |
|----------|---------|---------|
| `mode` | `kinematic` | `kinematic`: the joint velocity is set directly to the command. A perfect motor: exact, no tuning, no acceleration lag. `force`: a PID applies **torque** (`use_force_commands`), so the wheel accelerates gradually and can slip. |
| `p_gain` | 8.0 | Force mode only. Proportional: push harder the further the wheel speed is from the target. |
| `i_gain` | 0.1 | Force mode only. Integral: removes the small leftover speed error over time. |
| `d_gain` | 0.4 | Force mode only. Derivative: damps overshoot and oscillation. |
| `i_max` / `i_min` | 1.0 / -1.0 | Force mode only. Clamp on the integral term (stops wind-up). |
| `cmd_max` / `cmd_min` | 20.0 / -20.0 | Force mode only. Limits on the output force/torque. |

> The force-mode numbers are **placeholders** copied from the old caster controller. Real tuning needs the real robot mass and friction.

## `steering` — `front_left_steer_joint`, `front_right_steer_joint`, `caster_joint` (rear)

The command is an angle in **radians** (`/front_steer_cmd_pos`, `/castor_cmd_pos`). All three use a position PID. One shared section covers front and rear.

| Variable | Default | Meaning |
|----------|---------|---------|
| `p_gain` | 8.0 | Proportional: how hard the joint is pushed toward the target angle. |
| `i_gain` | 0.1 | Integral: removes a small remaining angle error. |
| `d_gain` | 0.4 | Derivative: damps overshoot so the wheel does not swing. |
| `i_max` / `i_min` | 1.0 / -1.0 | Clamp on the integral term. |
| `cmd_max` / `cmd_min` | 20.0 / -20.0 | Limits on the output effort. |

## Reminders
- The sim wheel does **not** replace the ESP32's PID. On hardware the ESP32 closes the speed loop; do not copy these gains to the firmware.
- Joint limits (for example ±90° steering) come from the **URDF**, not from this file.
