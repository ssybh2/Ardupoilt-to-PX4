# ArduPilot L1Quad to PX4 v1.17.0 Port

This repository stores the custom PX4 module used to port the L1 adaptive geometric controller from the L1Quad ArduPilot implementation into PX4 v1.17.0.

The repository name is currently `Ardupoilt-to-PX4`, but the intended project name is `ArduPilot-to-PX4`.

## References

- L1Quad repository: <https://github.com/sigma-pi/L1Quad>
- Original ArduPilot controller file: <https://github.com/sigma-pi/L1Quad/blob/main/L1AC_customization/ArduCopter/mode_adaptive.cpp>
- PX4 target version: `v1.17.0`

## What This Port Does

The original ArduPilot implementation directly computes motor PWM outputs. This PX4 port does not directly write PWM. Instead, it publishes PX4 control setpoints:

```text
PX4 estimator / simulated sensors
        -> l1_adaptive_control
        -> trajectory generator
        -> geometric controller
        -> L1 adaptive augmentation
        -> vehicle_thrust_setpoint / vehicle_torque_setpoint
        -> PX4 control_allocator
        -> actuator_motors / actuator_outputs
```

The current module includes:

- trajectory generation for takeoff and hold;
- geometric position and attitude control;
- L1 adaptive augmentation;
- PX4 `vehicle_thrust_setpoint` and `vehicle_torque_setpoint` publication;
- safety gates for invalid state, disarmed state, and failsafe;
- SIH validation using `sihsim_quadx`.

## Repository Layout

```text
l1_adaptive_control/
  CMakeLists.txt
  Kconfig
  L1AdaptiveControl.cpp
  L1AdaptiveControl.hpp
  GeometricController.cpp
  GeometricController.hpp
  TrajectoryGenerator.cpp
  TrajectoryGenerator.hpp
```

This repository is not a full PX4 fork. It contains the custom module that should be copied into a PX4 v1.17.0 checkout.

## Reproduce From A Fresh Checkout

### 1. Clone PX4 v1.17.0

```bash
mkdir -p ~/px4_ws
cd ~/px4_ws
git clone --recursive https://github.com/PX4/PX4-Autopilot.git PX4-Autopilot-v1.17.0
cd PX4-Autopilot-v1.17.0
git checkout v1.17.0
git submodule update --init --recursive
```

Install PX4 dependencies using the normal PX4 setup for your OS before building.

### 2. Clone This Module Repository

```bash
cd ~/px4_ws
git clone git@github.com:ssybh2/Ardupoilt-to-PX4.git
```

If SSH access is not configured, use HTTPS instead:

```bash
git clone https://github.com/ssybh2/Ardupoilt-to-PX4.git
```

### 3. Copy The Module Into PX4

```bash
cd ~/px4_ws/PX4-Autopilot-v1.17.0
mkdir -p src/modules/l1_adaptive_control
cp -r ../Ardupoilt-to-PX4/l1_adaptive_control/* src/modules/l1_adaptive_control/
```

### 4. Enable The Module In The SITL Board Config

Open:

```text
boards/px4/sitl/default.px4board
```

Add:

```text
CONFIG_MODULES_L1_ADAPTIVE_CONTROL=y
```

If your PX4 tree does not automatically expose the module in Kconfig, run:

```bash
make px4_sitl_default boardconfig
```

Then check that `modules/l1_adaptive_control` is enabled.

### 5. Build PX4 SITL

```bash
cd ~/px4_ws/PX4-Autopilot-v1.17.0
make px4_sitl_default
```

Expected result: the build completes and links `bin/px4`.

## No-UI SIH Validation

Gazebo UI is not required for the basic validation. The current verified path uses PX4's built-in SIH simulator:

```bash
cd ~/px4_ws/PX4-Autopilot-v1.17.0
make px4_sitl sihsim_quadx
```

In the PX4 shell, first verify that estimator topics exist:

```sh
listener vehicle_attitude 1
listener vehicle_local_position 1
listener vehicle_angular_velocity 1
listener vehicle_status 1
```

You should see valid attitude, local position, and angular velocity data.

## Run The L1 Controller In SIH

The current module publishes the same PX4 thrust and torque setpoint topics as the default multicopter rate controller. For this test, stop the default rate controller before starting the L1 module:

```sh
mc_rate_control stop
l1_adaptive_control start
commander arm
```

After a few seconds, inspect the L1 module:

```sh
l1_adaptive_control status
```

Expected output includes:

```text
state_valid_for_control=1
trajectory_valid=1
geometric_output_valid=1
l1_update_executed=1
publish_active=1
publish_count > 0
```

Inspect the PX4 setpoint topics:

```sh
listener vehicle_thrust_setpoint 1
listener vehicle_torque_setpoint 1
listener actuator_motors 1
listener vehicle_local_position 1
listener vehicle_attitude 1
```

Typical successful SIH output looks like:

```text
vehicle_thrust_setpoint.xyz = [0.00000, 0.00000, -0.50969]
vehicle_torque_setpoint.xyz = [0.01960, 0.00871, 0.00468]
actuator_motors.control = [0.50666, 0.52207, 0.52504, 0.48499]
vehicle_attitude ~= Roll -1.2 deg, Pitch -0.8 deg, Yaw -1.0 deg
vehicle_local_position.z ~= -1.2 m
```

This means the L1 controller is publishing PX4-compatible control setpoints, the control allocator is consuming them, and SIH is responding.

## One-Shot Test Command

You can run the same validation non-interactively:

```bash
cd ~/px4_ws/PX4-Autopilot-v1.17.0

timeout 90s bash -lc '(sleep 7; \
echo "mc_rate_control stop"; \
echo "l1_adaptive_control start"; \
sleep 1; \
echo "commander arm"; \
sleep 8; \
echo "l1_adaptive_control status"; \
echo "listener vehicle_thrust_setpoint 1"; \
echo "listener vehicle_torque_setpoint 1"; \
echo "listener actuator_motors 1"; \
echo "listener vehicle_local_position 1"; \
echo "listener vehicle_attitude 1"; \
echo "commander disarm"; \
echo "l1_adaptive_control stop"; \
echo "shutdown") | make px4_sitl sihsim_quadx'
```

## Notes And Limitations

- This is still a research/control-port module, not a production flight mode.
- Do not test on real hardware without additional safety review, parameterization, and actuator scaling validation.
- The module currently uses hard-coded SITL-style mass, inertia, gains, and setpoint normalization constants.
- The test intentionally stops `mc_rate_control` because both controllers publish `vehicle_thrust_setpoint` and `vehicle_torque_setpoint`.
- A cleaner production integration should give the L1 module explicit controller ownership through startup/airframe configuration instead of manually stopping `mc_rate_control`.
- Gazebo GUI is optional for this validation. On some ARM desktop systems, Gazebo Classic may fail in OGRE/OpenGL before PX4 starts. Use `sihsim_quadx` for no-UI validation.

## Development Commands

Build:

```bash
make px4_sitl_default
```

Run no-UI SIH:

```bash
make px4_sitl sihsim_quadx
```

Start and stop the module:

```sh
l1_adaptive_control start
l1_adaptive_control status
l1_adaptive_control stop
```

