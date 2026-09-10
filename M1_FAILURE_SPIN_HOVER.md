# M1 Failure + Spin-Hover Experiment (`shut_m1`)

This branch is dedicated to a single-motor-out quadrotor experiment on PX4 v1.17.0.

## 1. Current code entry points

### Keyboard node

`l1_keyboard_throttle/L1KeyboardThrottle.cpp`

- `w`: increase height-control stick by 0.2
- `s`: decrease height-control stick by 0.2
- `x` / space: return height-control stick to zero
- `0`: inject Motor 1 OFF failure
- `r`: restore Motor 1
- `q`: quit

The motor failure command is published as `VEHICLE_CMD_INJECT_FAILURE` with:

- `FAILURE_UNIT_SYSTEM_MOTOR`
- `FAILURE_TYPE_OFF` / `FAILURE_TYPE_OK`
- motor instance `1`

### PID baseline controller + L1 entry

`l1_adaptive_control/L1AdaptiveControl.cpp`

The main loop is:

```text
trajectory generator
    -> update_pid_input()
    -> run_pid_controller()
    -> run_l1_adaptive_augmentation()
    -> publish_control_setpoints()
```

The baseline controller is `SimplePIDController`. The old geometric-controller member and function names have been removed on this branch so the source now reflects the actual controller being executed.

The PID baseline controller outputs physical:

```text
[T, Mx, My, Mz]
```

The L1 adaptive layer adds its correction and the module publishes:

```text
vehicle_thrust_setpoint
vehicle_torque_setpoint
```

### PX4 mixer / allocator

PX4 v1.17 does not use the old text mixer as the normal multicopter path. The relevant mixer is:

```text
src/modules/control_allocator/ControlAllocator.cpp
```

The allocator converts `vehicle_thrust_setpoint` + `vehicle_torque_setpoint` into `actuator_motors`.

PX4 normally stops an injected motor but does not necessarily remove the injected stopped motor from the allocation effectiveness matrix. This branch therefore provides:

```text
patches/px4-v1.17.0-m1-spin-hover-control-allocation.patch
```

The patch does two experiment-specific things after a single motor is stopped:

1. removes the stopped motor from the effectiveness matrix;
2. zeros the yaw-effectiveness row, so the remaining motors prioritize collective thrust, roll and pitch.

## 2. Why yaw must be released

A normal quadrotor has four motor inputs and can command approximately:

```text
collective thrust + roll + pitch + yaw
```

After M1 is lost only three independent motor inputs remain. Trying to keep all four outputs controlled is over-constrained.

This branch therefore uses the standard spin-hover idea:

```text
keep:    thrust, roll, pitch
release: yaw angle / yaw torque
```

The vehicle is allowed to spin around the vertical axis while position/altitude and thrust-vector direction are controlled.

## 3. Spin-hover PID changes

`l1_adaptive_control/SimplePIDController.cpp`

The translational loop is:

```text
position error + velocity error + trajectory acceleration
    -> desired acceleration in NED
```

For horizontal control, desired roll/pitch are calculated using the **current yaw** rather than a fixed yaw target. This is essential because the body frame continuously rotates during spin-hover.

Yaw behavior is deliberately relaxed:

```text
desired_yaw = current_yaw
yaw attitude error = 0
yaw rate error = 0
Mz_PID = 0
```

The patched allocator also removes yaw from the control effectiveness matrix after the motor-out event, so any downstream L1 yaw correction cannot steal authority from thrust/roll/pitch.

## 4. Apply to a PX4 v1.17.0 checkout

From the PX4 tree:

```bash
cd ~/px4_ws/PX4-Autopilot-v1.17.0

git checkout v1.17.0

mkdir -p src/modules/l1_adaptive_control
cp -r ../Ardupoilt-to-PX4/l1_adaptive_control/* src/modules/l1_adaptive_control/

mkdir -p src/modules/l1_keyboard_throttle
cp -r ../Ardupoilt-to-PX4/l1_keyboard_throttle/* src/modules/l1_keyboard_throttle/

git apply ../Ardupoilt-to-PX4/patches/px4-v1.17.0-m1-spin-hover-control-allocation.patch
```

Enable the two modules in the board configuration as in the repository README.

## 5. Required PX4 parameters for failure injection

In the PX4 shell:

```sh
param set SYS_FAILURE_EN 1
param set CA_FAILURE_MODE 1
```

`CA_FAILURE_MODE=1` is required so the allocator processes the motor failure path.

## 6. Recommended SIH test sequence

Build and start SIH first:

```bash
make px4_sitl sihsim_quadx
```

Then in the PX4 shell:

```sh
mc_rate_control stop
l1_adaptive_control start
commander arm
```

Allow takeoff to finish and confirm normal hover. Then start the keyboard node:

```sh
l1_keyboard_throttle start
```

Press:

```text
0
```

to shut down M1.

Expected behavior after the transient:

- M1 is stopped;
- M2/M3/M4 remain active;
- yaw angle is not held;
- the vehicle develops a continuous yaw spin;
- roll/pitch and vertical thrust continue trying to hold the hover point.

Press `r` to restore M1.

## 7. Signals to inspect

Use these listeners during the test:

```sh
listener actuator_motors 1
listener control_allocator_status 1
listener vehicle_attitude 1
listener vehicle_angular_velocity 1
listener vehicle_local_position 1
listener vehicle_thrust_setpoint 1
listener vehicle_torque_setpoint 1
```

For Motor 1 failure, the allocator masks should indicate the first motor (bit 0), while the vehicle yaw rate should become non-zero and roll/pitch/position remain bounded.

## 8. Safety / scope

This is an experimental fault-tolerant control branch, not a validated real-flight configuration.

The PID gains, 3 kg mass, thrust normalization constants, inertia and L1 gains in this repository are still research/SITL values. Validate in SIH/SITL first. Do not move directly to propellers-on hardware testing without checking motor numbering, rotation direction, geometry parameters, actuator limits, estimator behavior during fast yaw spin and kill/disarm behavior.
