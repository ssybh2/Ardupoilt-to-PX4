# L1 Source-Equivalent Modularization Design

## Goal

Refactor the original `sigma-pi/L1Quad` `mode_adaptive.cpp` into independent PX4-compatible C++ modules without redesigning the controller mathematics. The refactor is a source-equivalent decomposition: equations, update order, state variables, fixed sample time, filters, trajectory formulas, and motor-mixing mathematics must follow the original implementation. PX4-specific code is limited to state acquisition, scheduling, logging/status, arming/failsafe gates, and final actuator publication.

## Source of truth

1. `sigma-pi/L1Quad/L1AC_customization/ArduCopter/mode_adaptive.cpp`
2. `sigma-pi/L1Quad/L1AC_customization/ArduCopter/ACRL_trajectories.cpp/.h`
3. The uploaded DSun `GeometricController.cpp/.hpp` supplied for this refactor.

When the current PX4 module differs from these sources, the source implementation wins unless the difference is strictly required to map an ArduPilot API onto PX4.

## Non-goals

- Do not invent alternative control laws or mathematically equivalent rewrites.
- Do not add adaptive-command saturation that is absent from the source implementation.
- Do not replace fixed `dt = 0.0025f` in the L1 predictor with measured/dynamic `dt`.
- Do not replace the ACRL trajectory equations with newly designed takeoff/hover/circle equations.
- Do not replace the source motor-mixing mathematics with a different mixer as part of the source-equivalent algorithm path.
- Do not introduce custom matrix algebra beyond helper functions already present in the original source (`unit_vec`, `hatOperator`, `veeOperator`, `mat4Inv`, etc.).

## Module boundaries

### `ACRLTrajectories.cpp/.hpp`

Direct PX4-type port of the original `ACRL_trajectories.cpp/.h`. Preserve the original polynomial coefficients, circle/figure-eight progression, yaw representation as `Vector2f [cos(yaw), sin(yaw)]`, landing phases, and polynomial helper functions. Replace ArduPilot-only GCS messages with optional PX4 logging only; do not change numerical outputs.

### `TrajectoryGenerator.cpp/.hpp`

Thin orchestration wrapper around the ACRL trajectory functions. Preserve the original `ModeAdaptive::run()` selection logic: first two seconds use takeoff; afterwards dispatch by trajectory index; preserve SITL/real circle offsets and landing override behavior. It must not contain independent trajectory equations.

### `GeometricController.cpp/.hpp`

Use the uploaded DSun implementation as the interface baseline, with compilation defects corrected. Preserve the original `geometricController()` equation order and original helper-function mathematics. Inputs contain PX4 state plus `Vector2f`-equivalent target yaw, yaw derivative and yaw second derivative. Output is the original four-channel baseline command `[thrust, Mx, My, Mz]` plus minimal diagnostics.

### `L1AdaptiveAugmentation.cpp/.hpp`

Direct modular extraction of `ModeAdaptive::L1AdaptiveAugmentation()`. Preserve fixed `dt = 0.0025f`, state predictor equations, piecewise-constant uncertainty estimate, matched/unmatched uncertainty decomposition, LPF1/LPF2 equations, negation, state-update order, and `l1enable` multiplication. Parameters hold the original `As_v`, `As_omega`, cutoff frequencies, mass, `J`, and `Jinv`; no additional limiter or altered estimator is allowed.

### `MotorMixer.cpp/.hpp`

Direct modular extraction of `motorMixing()`, `iterativeMotorMixing()`, `mat4Inv()` and the associated source helper mathematics. Preserve original SITL and real motor-model constants. Output remains four source-style motor commands in the source 0..100 convention before the source saturation step.

### `L1AdaptiveControl.cpp/.hpp`

PX4 adapter/orchestrator only. It obtains PX4 state, executes the source order `trajectory -> geometricController -> L1AdaptiveAugmentation -> motorMixing`, applies the source 0..100 motor saturation, and maps the resulting motor commands to the PX4 actuator publication interface. It must not contain replacement control mathematics.

## Parameter/configuration parity

The design must retain the two original vehicle configurations rather than blending them:

- SITL: mass and inertia/motor constants from the original `!REAL_OR_SITL` branch.
- Real: mass, inertia, gains and motor constants from the original `REAL_OR_SITL` branch; the uploaded DSun geometric-controller defaults correspond to this real-aircraft configuration.

A module configuration flag/parameter may select the source configuration, but it may only select between source values; it may not synthesize new values.

## Testing

Tests become source-parity tests. For deterministic inputs they check the same intermediate/output quantities that the source equations produce:

- ACRL takeoff polynomial and representative circle/figure-eight/landing outputs.
- Geometric controller hover and nontrivial trajectory command using the uploaded DSun/original equation order.
- L1 fixed-`dt` predictor, uncertainty estimate, two-stage filtering, and reset/initialization state.
- Motor mixer output for representative thrust/moment commands.
- Pipeline order and type/interface compatibility.

The tests may add safety checks around invalid PX4 state, but those checks must live outside the numerical source-equivalent core.

## Acceptance criteria

1. No current custom cubic trajectory remains in the core path; ACRL source equations are used.
2. L1 uses the original fixed 400 Hz discrete equations and has no custom adaptive saturation.
3. Geometric control follows the uploaded DSun/original equations without alternative matrix/control formulations.
4. Original motor mixer mathematics exists as a standalone module and is used by the source-equivalent path.
5. Headers and implementations are complete and compile together in a PX4 v1.17 module checkout.
6. Unit tests document source-equivalent numerical behavior and protect against later algorithm drift.
