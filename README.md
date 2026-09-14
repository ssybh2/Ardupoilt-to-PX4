# ArduPilot L1Quad to PX4 v1.17.0 — source-equivalent modular port

This repository contains a PX4 v1.17.0 modular port of the adaptive flight controller in:

- <https://github.com/sigma-pi/L1Quad/blob/main/L1AC_customization/ArduCopter/mode_adaptive.cpp>
- <https://github.com/sigma-pi/L1Quad/blob/main/L1AC_customization/ArduCopter/ACRL_trajectories.cpp>

The purpose of this branch is **not** to redesign the controller for PX4. The purpose is to split the original ArduPilot implementation into independent `.hpp/.cpp` modules while preserving the original equations, state-update order, trajectory mathematics, L1 discrete-time implementation, and iterative motor mixer.

The repository name is currently `Ardupoilt-to-PX4`; the intended project name is `ArduPilot-to-PX4`.

## Source-equivalent control chain

The numerical path is now:

```text
PX4 estimator / simulated sensors
        |
        v
TrajectoryGenerator
        |
        | calls the original ACRL trajectory equations
        v
GeometricController
        |
        | u_b = [F, Mx, My, Mz]
        v
L1AdaptiveAugmentation
        |
        | u_ad
        v
u_total = u_b + u_ad
        |
        v
MotorMixer
        |
        | original motorMixing()
        | original iterativeMotorMixing()
        | original mat4Inv()
        v
source motor commands 0..100
        |
        | PX4 interface-only mapping /100
        v
actuator_motors.control[0..3]
        |
        v
PX4 ESC / simulator output path
```

This deliberately differs from the earlier branch implementation, which ended the custom controller at `vehicle_thrust_setpoint` / `vehicle_torque_setpoint` and let PX4 `control_allocator` perform different motor allocation mathematics.

## Exact source-to-module mapping

```text
l1_adaptive_control/
  L1SourceConfig.hpp
      original REAL_OR_SITL selection (default 0 = SITL)

  ACRLTrajectories.cpp/.hpp
      ACRL_trajectory_takeoff()
      ACRL_trajectory_transition_to_start()
      ACRL_trajectory_circle_variable_yaw()
      ACRL_trajectory_circle_fixed_yaw()
      ACRL_trajectory_figure8_fixed_yaw()
      ACRL_trajectory_figure8_tilted()
      ACRL_trajectory_land()
      polyEval / derivative helpers

  TrajectoryGenerator.cpp/.hpp
      only the trajectory-selection, timing and landing orchestration
      that originally lived in ModeAdaptive::run()

  GeometricController.cpp/.hpp
      original geometricController() mathematics
      using the uploaded DSun interface as the baseline
      with only PX4 type/API and compile fixes

  L1AdaptiveAugmentation.cpp/.hpp
      original L1AdaptiveAugmentation()
      fixed dt = 0.0025 s
      original state predictor
      original PhiInvmu expressions
      original matched / unmatched uncertainty estimate
      original LPF1 / LPF2 equations
      original u_ad = -u_ad and l1enable handling

  MotorMixer.cpp/.hpp
      original motorMixing()
      original iterativeMotorMixing()
      original mat4Inv()

  L1AdaptiveControl.cpp/.hpp
      PX4 adapter and source-order orchestrator only
```

## What was intentionally removed from the numerical core

The earlier PX4 rewrite contained behavior that does not exist in `mode_adaptive.cpp`. It is not part of the source-equivalent path anymore:

- timestamp-derived L1 `dt`;
- custom `MAX_L1_*` adaptive-command limits;
- custom `phi_inverse_mu()` rewrite;
- newly designed cubic takeoff trajectory;
- custom hover/circle trajectory equations;
- RC throttle height takeover inside trajectory generation;
- replacement motor allocation through `vehicle_thrust_setpoint` / `vehicle_torque_setpoint` + `control_allocator`.

PX4-specific checks for invalid estimator state, disarmed state and failsafe remain outside the numerical core as interface/safety gates.

## Original configuration values

`L1SourceConfig.hpp` preserves the original build-time convention:

```cpp
#ifndef REAL_OR_SITL
#define REAL_OR_SITL 0
#endif
```

`0` selects the original SITL values; `1` selects the original real-aircraft values.

### SITL (`REAL_OR_SITL = 0`)

The module uses the original L1Quad SITL values, including:

```text
mass = 3.0 kg
J = diag(0.023, 0.023, 0.0459) kg m^2
Jinv = diag(43.478, 43.478, 21.786)
As_v = -5
As_omega = -10
LPF1 thrust = 10
LPF1 moment = 10
LPF2 moment = 2
```

The original SITL geometric-controller gains are also loaded by `L1AdaptiveControl`.

### Real (`REAL_OR_SITL = 1`)

The geometric-controller defaults match the uploaded DSun controller / original real vehicle:

```text
mass = 0.62 kg
J = diag(0.002016, 0.001827, 0.00322) kg m^2
Jinv = diag(496.03, 547.345, 310.559)
```

The DSun gain set and original real L1 filter constants are retained.

## Important: original L1 enable semantics

The upstream repository defines `L1ENABLE_DEFAULT = 0`. This port preserves that default instead of silently forcing L1 on.

After starting the module, enable adaptive augmentation explicitly when desired:

```sh
l1_adaptive_control l1 enable
```

Check it with:

```sh
l1_adaptive_control l1 status
```

Disable it with:

```sh
l1_adaptive_control l1 disable
```

With L1 disabled, the L1 predictor/estimator still follows the source update equations, but the returned adaptive command is multiplied by zero exactly as in the original implementation.

## Original trajectory-index semantics

The source `TRAJINDEX` meanings are preserved:

```text
0  source default / hover after takeoff
1  circle with variable yaw
2  circle with fixed yaw
3  figure-eight with fixed yaw
4  tilted figure-eight
```

Set the index with:

```sh
l1_adaptive_control trajectory 2
```

Check it with:

```sh
l1_adaptive_control trajectory status
```

The original `LandFlag` concept is exposed as:

```sh
l1_adaptive_control land enable
l1_adaptive_control land status
l1_adaptive_control land disable
```

## Unit tests

The `*Test.cpp` files are now intended as **source-parity tests**, not tests for an alternative PX4 controller design.

```text
TrajectoryGeneratorTest.cpp
    verifies the original ACRL polynomial / trajectory behavior

GeometricControllerTest.cpp
    verifies the original yaw-vector API and geometric-controller outputs

L1AdaptiveAugmentationTest.cpp
    verifies fixed dt=0.0025, source initialization/update ordering,
    source filtering, and absence of the previous custom L1 limiter

MotorMixerTest.cpp
    verifies representative outputs from the original three-pass mixer
```

The test targets are registered in `l1_adaptive_control/CMakeLists.txt` with `px4_add_unit_gtest`.

## Copy into PX4 v1.17.0

```bash
mkdir -p ~/px4_ws
cd ~/px4_ws

git clone --recursive https://github.com/PX4/PX4-Autopilot.git PX4-Autopilot-v1.17.0
cd PX4-Autopilot-v1.17.0
git checkout v1.17.0
git submodule update --init --recursive

cd ~/px4_ws
git clone https://github.com/ssybh2/Ardupoilt-to-PX4.git
cd Ardupoilt-to-PX4
git checkout refactor/l1-adaptive-augmentation

cd ~/px4_ws/PX4-Autopilot-v1.17.0
mkdir -p src/modules/l1_adaptive_control
cp -r ../Ardupoilt-to-PX4/l1_adaptive_control/* src/modules/l1_adaptive_control/
```

Enable the module in:

```text
boards/px4/sitl/default.px4board
```

with:

```text
CONFIG_MODULES_L1_ADAPTIVE_CONTROL=y
```

Then build:

```bash
make px4_sitl_default
```

## Run the source-equivalent module

The module now publishes `actuator_motors` directly because the original source has already performed its own motor mixing. Therefore **do not leave another PX4 motor allocator publishing the same topic during this isolated validation**.

For a controlled SIH/SITL test, stop the default competing controller/allocation path first:

```sh
mc_rate_control stop
control_allocator stop
l1_adaptive_control start
```

Select a trajectory and enable L1 if desired:

```sh
l1_adaptive_control trajectory 2
l1_adaptive_control l1 enable
```

Then arm:

```sh
commander arm
```

Inspect:

```sh
l1_adaptive_control status
listener actuator_motors 1
listener vehicle_local_position 1
listener vehicle_attitude 1
listener vehicle_angular_velocity 1
```

The four source motor commands are saturated to the original `0..100` range and then mapped to PX4's normalized `actuator_motors.control` range by dividing by 100. Unused motor channels are published as `NAN`, as required by the PX4 actuator message.

## Build / test commands

From the PX4 v1.17.0 checkout after copying the module:

```bash
make px4_sitl_default
make tests TESTFILTER=TrajectoryGenerator
make tests TESTFILTER=GeometricController
make tests TESTFILTER=L1AdaptiveAugmentation
make tests TESTFILTER=MotorMixer
```

## Design and implementation notes

The branch contains the written source-equivalence design and implementation plan:

```text
docs/superpowers/specs/2026-09-14-l1-source-equivalent-modularization-design.md
docs/superpowers/plans/2026-09-14-l1-source-equivalent-modularization.md
```

## Safety / integration note

This is a research controller port, not a production flight-mode integration. Direct `actuator_motors` publication is intentionally used to preserve the original `motorMixing()` result. Real-hardware use still requires explicit ownership of the actuator topic, output ordering verification, ESC scaling verification, arming/failsafe integration review, and controlled bench testing before flight.

The separate `l1_keyboard_throttle` directory remains in the repository as legacy experimental tooling, but it is no longer part of the strict `mode_adaptive.cpp` numerical path.
