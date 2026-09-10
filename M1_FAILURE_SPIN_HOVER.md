# `shut_m1`: PID + L1 + M1 Failure Spin-Hover

This branch is the PX4 v1.17.0 experiment for keyboard-controlled hover, automatic landing and single-M1 failure spin-hover.

## Keyboard workflow

`l1_keyboard_throttle` now uses:

```text
1        arm + take off from the current point + hover at 1 m
2        automatic smooth landing; disarm after PX4 confirms landed
w        increase height command
s        decrease height command
x/space  return height stick to zero and hold the current height reference
0        inject Motor 1 OFF failure
r        restore Motor 1
q        quit the keyboard node
```

The `w/s` value is a vertical-height/velocity command to the trajectory generator. It is not direct raw motor PWM.

## Trajectory

`l1_adaptive_control/TrajectoryGenerator.cpp` no longer generates the old circle trajectory.

The active state flow is:

```text
WaitForValidState
      |
      | keyboard 1
      v
   Takeoff
      |
      v
    Hover  <---- w/s modifies the hover altitude
      |
      | keyboard 2
      v
   Landing
      |
      v
    Landed
```

Key `1` is latched even if PX4 Commander is still completing the arm request. The takeoff start point and ground reference are captured from the current local NED position. The default target is 1 m above that captured point and the takeoff transition lasts 2 s.

Key `2` captures the current position and starts a 4 s fifth-order smoothstep descent to the captured ground plane. During landing, `w/s` cannot override the descent. After the descent, the target is biased slightly below the ground reference so the vertical loop reduces thrust while PX4's land detector confirms touchdown. The keyboard node then sends a normal `VEHICLE_CMD_COMPONENT_ARM_DISARM` disarm request.

The historical `CommandedMode::Circle` enum value is retained only so the existing L1 wrapper still compiles; `TrajectoryGenerator::set_commanded_mode()` maps it back to hover and no circle motion is produced.

## Controller path

```text
TrajectoryGenerator
    -> SimplePIDController
    -> L1 adaptive augmentation
    -> vehicle_thrust_setpoint / vehicle_torque_setpoint
    -> PX4 control_allocator
    -> actuator_motors
```

The PID branch uses yaw-relaxed control for the M1 spin-hover experiment. Horizontal NED correction is transformed with the current yaw, so roll/pitch corrections remain aligned with the world-frame position error while the body spins.

## M1 failure

Key `0` still publishes `VEHICLE_CMD_INJECT_FAILURE` for Motor instance 1. Apply:

```text
patches/px4-v1.17.0-m1-spin-hover-control-allocation.patch
```

to the PX4 v1.17.0 tree. The patch removes the stopped motor from the effectiveness matrix and removes yaw authority after a single motor failure so the remaining three motors prioritize:

```text
collective thrust + roll + pitch
```

Yaw is intentionally released and the vehicle is allowed to spin.

Required PX4 parameters for the failure-injection path:

```sh
param set SYS_FAILURE_EN 1
param set CA_FAILURE_MODE 1
```

## Recommended SIH sequence

```sh
mc_rate_control stop
l1_adaptive_control start
l1_keyboard_throttle start
```

Then use the keyboard:

```text
1   -> takeoff and hover
w/s -> change hover altitude
0   -> fail M1 and enter the spin-hover experiment
2   -> land from the current hover point
```

Useful listeners:

```sh
listener actuator_motors 1
listener control_allocator_status 1
listener vehicle_land_detected 1
listener vehicle_attitude 1
listener vehicle_angular_velocity 1
listener vehicle_local_position 1
listener vehicle_thrust_setpoint 1
listener vehicle_torque_setpoint 1
```

## Scope

This remains an experimental fault-tolerant controller. The PID gains, 3 kg mass, thrust normalization, inertia and L1 gains are research/SIH values. Validate the complete workflow in SIH/SITL before any propellers-on hardware test, especially the M1 numbering, motor rotation directions, allocator geometry, fast-yaw estimator behavior and landing/disarm behavior.
