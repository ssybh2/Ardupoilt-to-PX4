# L1 Source-Equivalent Modularization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split L1Quad `mode_adaptive.cpp` into PX4-compatible modules while preserving the original numerical algorithms and update order.

**Architecture:** Keep PX4-specific state acquisition and actuator publication in `L1AdaptiveControl`, but extract the original ACRL trajectory equations, geometric controller, L1 augmentation, and motor mixer into focused classes/functions. The numerical core follows the L1Quad source literally; only API/type adaptation and necessary compile fixes are permitted.

**Tech Stack:** PX4 v1.17.0, C++17/PX4 toolchain, `matrix` library, uORB, GoogleTest via `px4_add_unit_gtest`.

**Spec:** `docs/superpowers/specs/2026-09-14-l1-source-equivalent-modularization-design.md`

## Global Constraints

- `sigma-pi/L1Quad` `mode_adaptive.cpp` and `ACRL_trajectories.cpp` are the mathematical source of truth.
- Use the uploaded DSun geometric controller as the geometric-controller baseline, correcting only compile/interface defects.
- Preserve original fixed `dt = 0.0025f` for L1.
- No custom adaptive-command saturation or replacement estimator equations.
- No replacement trajectory equations.
- Preserve original motor-mixing equations and source order.

---

### Task 1: Add source-parity trajectory module and tests

**Files:**
- Create: `l1_adaptive_control/ACRLTrajectories.hpp`
- Create: `l1_adaptive_control/ACRLTrajectories.cpp`
- Modify: `l1_adaptive_control/TrajectoryGenerator.hpp`
- Modify: `l1_adaptive_control/TrajectoryGenerator.cpp`
- Modify: `l1_adaptive_control/TrajectoryGeneratorTest.cpp`

**Interfaces:**
- Consumes: time in current run, trajectory index, radii, target speed, current landing state.
- Produces: target position/velocity/acceleration/jerk/snap and `Vector2f` yaw/yaw derivatives exactly matching ACRL source semantics.

- [ ] **Step 1: Replace trajectory tests with source-parity assertions**

Add tests for the original takeoff polynomial at `t=0`, `t=1`, and just before `t=2`, plus fixed-yaw circle initial output. Tests must assert the original polynomial coefficients and yaw-vector representation.

- [ ] **Step 2: Run trajectory test to verify the current implementation fails source parity**

Run in a PX4 v1.17 checkout after copying the module:

```bash
make tests TESTFILTER=TrajectoryGenerator
```

Expected: at least the source takeoff-polynomial assertion fails because the current implementation uses a cubic trajectory rather than the original seventh-order polynomial.

- [ ] **Step 3: Port `ACRL_trajectories.cpp/.h` literally into `ACRLTrajectories.cpp/.hpp`**

Use PX4 `matrix::Vector3f`/`Vector2f` types and `sinf/cosf/powf/sqrtf`, preserve all source coefficients and formulas. Remove only ArduPilot GCS calls or replace them with non-numerical PX4 log calls.

- [ ] **Step 4: Reduce `TrajectoryGenerator` to source dispatch logic**

Implement the original `timeInThisRun < 2` and `trajIndex` switch logic, plus landing override. Do not retain independent polynomial/circle equations.

- [ ] **Step 5: Run trajectory tests**

```bash
make tests TESTFILTER=TrajectoryGenerator
```

Expected: PASS.

---

### Task 2: Replace geometric controller with DSun/source-equivalent implementation

**Files:**
- Modify: `l1_adaptive_control/GeometricController.hpp`
- Modify: `l1_adaptive_control/GeometricController.cpp`
- Modify: `l1_adaptive_control/GeometricControllerTest.cpp`

**Interfaces:**
- Consumes: PX4 NED state, body-to-NED quaternion, body angular velocity, target derivatives, target yaw vectors, source controller parameters.
- Produces: `[target_thrust, Mx, My, Mz]`.

- [ ] **Step 1: Add tests for source yaw-vector API and original hover command**

Use `target_yaw={1,0}`, zero derivatives, matched actual/target state, and assert `target_thrust=m*g` and moments near zero.

- [ ] **Step 2: Run test to verify current API/implementation fails the new source-compatible interface**

```bash
make tests TESTFILTER=GeometricController
```

Expected: compile/test failure until the yaw vector interface and DSun implementation are installed.

- [ ] **Step 3: Install the uploaded DSun controller and fix compile-only defects**

Correct `target_yaw_ddot` to a two-element array, correct repeated/wrong indexes, remove stray parenthesis, map quaternion/state arrays into PX4 matrix types, and keep the original equation order. Preserve original `unit_vec`, `hatOperator`, and `veeOperator` mathematics.

- [ ] **Step 4: Run geometric controller tests**

```bash
make tests TESTFILTER=GeometricController
```

Expected: PASS.

---

### Task 3: Restore literal L1 adaptive augmentation

**Files:**
- Modify: `l1_adaptive_control/L1AdaptiveAugmentation.hpp`
- Modify: `l1_adaptive_control/L1AdaptiveAugmentation.cpp`
- Modify: `l1_adaptive_control/L1AdaptiveAugmentationTest.cpp`

**Interfaces:**
- Consumes: current velocity, angular velocity, body-to-NED rotation, source baseline command, source parameters.
- Produces: source `u_ad` only; the orchestrator performs `u_b + u_ad` exactly as `ModeAdaptive::run()` does.

- [ ] **Step 1: Add source-parity tests for fixed `dt` and unbounded source filter output**

Tests must expose that the algorithm always uses `0.0025f` and that no custom `ADAPTIVE_LIMITS` clipping occurs.

- [ ] **Step 2: Run L1 tests and observe failure against current implementation**

```bash
make tests TESTFILTER=L1AdaptiveAugmentation
```

Expected: source-parity test fails because current code uses timestamp-derived `dt` and custom limits.

- [ ] **Step 3: Port `ModeAdaptive::L1AdaptiveAugmentation()` literally**

Keep `v_hat_prev`, `omega_hat_prev`, `v_prev`, `omega_prev`, `R_prev`, `u_b_prev`, `u_ad_prev`, `sigma_m_hat_prev`, `sigma_um_hat_prev`, `lpf1_prev`, and `lpf2_prev` as persistent class state. Preserve the original update ordering and direct `PhiInvmu` expressions.

- [ ] **Step 4: Run L1 tests**

```bash
make tests TESTFILTER=L1AdaptiveAugmentation
```

Expected: PASS.

---

### Task 4: Extract original motor mixer and tests

**Files:**
- Create: `l1_adaptive_control/MotorMixer.hpp`
- Create: `l1_adaptive_control/MotorMixer.cpp`
- Create: `l1_adaptive_control/MotorMixerTest.cpp`

**Interfaces:**
- Consumes: source `[thrust, Mx, My, Mz]` command and source SITL/real configuration.
- Produces: four source motor commands before 0..100 saturation.

- [ ] **Step 1: Add deterministic mixer tests**

Use hover thrust with zero moments to assert four equal outputs and use one nonzero moment case to assert the expected source X-layout sign pattern.

- [ ] **Step 2: Run mixer test and verify it fails because the module does not yet exist**

```bash
make tests TESTFILTER=MotorMixer
```

Expected: build/test target missing/failing.

- [ ] **Step 3: Copy `motorMixing()`, `iterativeMotorMixing()`, `mat4Inv()` and helpers into the new module**

Do not replace these equations with PX4 control-allocation mathematics.

- [ ] **Step 4: Run mixer tests**

```bash
make tests TESTFILTER=MotorMixer
```

Expected: PASS.

---

### Task 5: Rewire the PX4 orchestrator and build files

**Files:**
- Modify: `l1_adaptive_control/L1AdaptiveControl.hpp`
- Modify: `l1_adaptive_control/L1AdaptiveControl.cpp`
- Modify: `l1_adaptive_control/CMakeLists.txt`
- Modify: `README.md`

**Interfaces:**
- Consumes: uORB state/status.
- Produces: source-order motor outputs mapped to PX4 actuator output format.

- [ ] **Step 1: Add/adjust integration assertions where practical**

Ensure the module data flow is `trajectory -> geometric -> L1 -> sum -> motor mixer` and remove the current assumption that the numerical core ends at `vehicle_thrust_setpoint`/`vehicle_torque_setpoint`.

- [ ] **Step 2: Rewire `L1AdaptiveControl::Run()` to the source order**

Use the PX4 adapter only to populate source inputs and publish final motor commands. Keep arming/failsafe checks around, not inside, the numerical core.

- [ ] **Step 3: Update CMake**

Add `ACRLTrajectories.cpp`, `MotorMixer.cpp`, and `MotorMixerTest.cpp`; retain the three existing source-parity test targets.

- [ ] **Step 4: Build and run unit tests in PX4 v1.17**

```bash
make px4_sitl_default
make tests TESTFILTER=TrajectoryGenerator
make tests TESTFILTER=GeometricController
make tests TESTFILTER=L1AdaptiveAugmentation
make tests TESTFILTER=MotorMixer
```

Expected: build succeeds and all four test groups pass.

- [ ] **Step 5: Update README architecture description**

Document that the module is a source-equivalent decomposition of L1Quad and list the exact source-to-module correspondence.
