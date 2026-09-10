# Simple PID attitude baseline experiment

This branch replaces the **baseline geometric controller used by `l1_adaptive_control`** with a deliberately simple Euler-angle PID controller while keeping the surrounding PX4/L1 pipeline unchanged.

## Main branch baseline

On `main`, the control path is:

```text
TrajectoryGenerator
  -> GeometricController
     -> thrust [N]
     -> body moment [N m]
  -> L1 adaptive augmentation
  -> normalized vehicle_thrust_setpoint / vehicle_torque_setpoint
  -> PX4 control_allocator
  -> motors
```

The geometric attitude controller computes an SO(3) rotation error from the current and desired rotation matrices, then applies rotation/rate feedback plus feed-forward and gyroscopic compensation.

## This branch

On `experiment/simple-pid-attitude`, the path is:

```text
TrajectoryGenerator
  -> position/velocity PD
  -> desired NED acceleration
  -> small-angle roll/pitch setpoint + yaw setpoint
  -> Euler attitude PID + body-rate damping
  -> thrust [N] + body moment [N m]
  -> L1 adaptive augmentation
  -> normalized vehicle_thrust_setpoint / vehicle_torque_setpoint
  -> PX4 control_allocator
  -> motors
```

The old `GeometricController.*` files are intentionally left in the repository for side-by-side reference, but they are no longer part of the module build on this branch.

## Core PID law

For each attitude axis:

```text
angle_error = angle_setpoint - angle
rate_error  = rate_setpoint - body_rate

moment = Kp * angle_error
       + Ki * integral(angle_error)
       + Kd * rate_error
```

Yaw error is wrapped to `[-pi, pi]`. The integral term is clamped.

Initial attitude gains are defined at the top of `l1_adaptive_control/SimplePIDController.cpp`:

```cpp
ATT_KP = {5.4, 5.4, 0.092}
ATT_KI = {0.15, 0.15, 0.01}
ATT_KD = {0.6, 0.6, 0.023}
```

These are starting values for SITL only and must be tuned before real flight.

## Important limitation

This is a **small-angle Euler controller** intended for simple hover/low-angle testing. It is not intended to match the large-angle/aggressive-flight behavior of the SO(3) geometric controller.

The L1 adaptive augmentation is still enabled in `L1AdaptiveControl.cpp`. Therefore the final published torque/thrust is:

```text
PID baseline + L1 adaptive correction
```

To evaluate the PID baseline by itself, set `L1_ENABLE = false` in `L1AdaptiveControl.cpp` before building.

## Build/test workflow

Copy this branch's `l1_adaptive_control` directory into the PX4 v1.17.0 tree as described in the main README, then build:

```bash
make px4_sitl_default
```

For no-UI SIH validation:

```bash
make px4_sitl sihsim_quadx
```

In the PX4 shell:

```sh
mc_rate_control stop
l1_adaptive_control start
commander arm
```

Start with hover testing. Do not begin with the circle trajectory. Inspect:

```sh
listener vehicle_thrust_setpoint 1
listener vehicle_torque_setpoint 1
listener actuator_motors 1
listener vehicle_attitude 1
listener vehicle_local_position 1
```

Disarm immediately if the attitude diverges or motor outputs saturate.
