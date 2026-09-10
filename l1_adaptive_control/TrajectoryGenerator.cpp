#include "TrajectoryGenerator.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

namespace
{

void copy3(const float in[3], float out[3])
{
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}

}

bool TrajectoryGenerator::update(const Input &input, Output &output)
{
	_last_input = input;
	output = Output{};
	output.timestamp_us = input.timestamp_us;
	set_zero_derivatives(output);

	// Keyboard commands are inspected before the armed gate so a takeoff request
	// can be latched while PX4 Commander is still completing the arm request.
	handle_keyboard_command(input);

	if (!input.state_valid_for_control || input.failsafe) {
		reset();
		_takeoff_requested = false;
		_landing_requested = false;

		copy3(input.current_position_ned, output.position_ned);
		output.yaw = input.current_yaw;
		output.mode = Mode::WaitForValidState;
		output.valid = false;
		_last_output = output;
		return true;
	}

	if (!input.armed) {
		// Keep a keyboard-1 request latched across the short arm transition, but
		// clear all active trajectory state while the vehicle is disarmed.
		reset();
		copy3(input.current_position_ned, output.position_ned);
		output.yaw = input.current_yaw;
		output.mode = Mode::WaitForValidState;
		output.valid = false;
		_last_output = output;
		return true;
	}

	if (!_initialized) {
		if (!_takeoff_requested) {
			copy3(input.current_position_ned, output.position_ned);
			output.yaw = input.current_yaw;
			output.mode = Mode::WaitForValidState;
			output.valid = false;
			_last_output = output;
			return true;
		}

		initialize_takeoff(input);
	}

	if (_landing_requested) {
		if (!_landing_initialized) {
			initialize_landing(input);
		}

		update_landing_target(input, output);

	} else {
		const float elapsed_s = math::max((input.timestamp_us - _takeoff_start_time_us) * 1e-6f, 0.f);
		output.elapsed_time_s = elapsed_s;

		if (elapsed_s < TAKEOFF_DURATION_S) {
			update_takeoff_target(input, output);

		} else {
			update_hover_target(input, output);
		}
	}

	_last_update_us = input.timestamp_us;
	output.valid = true;
	_last_output = output;
	return true;
}

void TrajectoryGenerator::handle_keyboard_command(const Input &input)
{
	if (!input.manual_height_control_enabled || !input.manual_height_control_valid) {
		return;
	}

	if (fabsf(input.manual_height_stick - KEYBOARD_TAKEOFF_COMMAND_STICK) < MANUAL_COMMAND_EPS) {
		// Start a new flight only when no active trajectory exists. If key 1 is
		// pressed during hover, simply keep hovering and cancel a pending landing.
		if (!_initialized) {
			_takeoff_requested = true;
		}

		_landing_requested = false;
		_landing_initialized = false;
		return;
	}

	if (fabsf(input.manual_height_stick - KEYBOARD_LAND_COMMAND_STICK) < MANUAL_COMMAND_EPS) {
		_takeoff_requested = false;

		if (_initialized) {
			_landing_requested = true;
		}
	}
}

void TrajectoryGenerator::initialize_takeoff(const Input &input)
{
	copy3(input.current_position_ned, _ground_position_ned);
	copy3(input.current_position_ned, _takeoff_start_position_ned);

	_takeoff_target_position_ned[0] = _takeoff_start_position_ned[0];
	_takeoff_target_position_ned[1] = _takeoff_start_position_ned[1];
	_takeoff_target_position_ned[2] = _ground_position_ned[2] - TAKEOFF_HEIGHT_M;
	copy3(_takeoff_target_position_ned, _hover_position_ned);

	_start_yaw = input.current_yaw;
	_takeoff_start_time_us = input.timestamp_us;
	_last_update_us = input.timestamp_us;
	_manual_hold_initialized = false;
	_landing_requested = false;
	_landing_initialized = false;
	_takeoff_requested = false;
	_initialized = true;
}

void TrajectoryGenerator::initialize_landing(const Input &input)
{
	copy3(input.current_position_ned, _landing_start_position_ned);
	copy3(_landing_start_position_ned, _landing_target_position_ned);
	_landing_target_position_ned[2] = _ground_position_ned[2];
	_landing_yaw = input.current_yaw;
	_landing_start_time_us = input.timestamp_us;
	_landing_initialized = true;
	_manual_hold_initialized = false;
}

void TrajectoryGenerator::reset()
{
	_initialized = false;
	_landing_initialized = false;
	_takeoff_start_time_us = 0;
	_landing_start_time_us = 0;
	_last_update_us = 0;
	_manual_hold_initialized = false;
}

void TrajectoryGenerator::set_commanded_mode(CommandedMode mode)
{
	// Circle flight was intentionally removed from shut_m1. Keep this method so
	// the existing L1 wrapper remains source-compatible; every legacy commanded
	// mode resolves to hover and cannot start a flight by itself.
	(void)mode;
	_commanded_mode = CommandedMode::Hover;
}

void TrajectoryGenerator::set_zero_derivatives(Output &output)
{
	for (int i = 0; i < 3; i++) {
		output.velocity_ned[i] = 0.f;
		output.acceleration_ned[i] = 0.f;
		output.jerk_ned[i] = 0.f;
		output.snap_ned[i] = 0.f;
	}

	output.yaw_rate = 0.f;
	output.yaw_accel = 0.f;
}

void TrajectoryGenerator::set_hold_position(Output &output, const float position_ned[3])
{
	copy3(position_ned, output.position_ned);
	set_zero_derivatives(output);
}

void TrajectoryGenerator::update_takeoff_target(const Input &input, Output &output)
{
	output.mode = Mode::Takeoff;
	output.yaw = _start_yaw;
	_manual_hold_initialized = false;

	const float elapsed_s = math::max((input.timestamp_us - _takeoff_start_time_us) * 1e-6f, 0.f);
	const float s = math::constrain(elapsed_s / TAKEOFF_DURATION_S, 0.f, 1.f);

	// Smooth cubic takeoff: h(s) = 3 s^2 - 2 s^3.
	const float h = 3.f * s * s - 2.f * s * s * s;
	const float h_dot = (6.f * s - 6.f * s * s) / TAKEOFF_DURATION_S;
	const float h_ddot = (6.f - 12.f * s) / (TAKEOFF_DURATION_S * TAKEOFF_DURATION_S);
	const float h_jerk = -12.f / (TAKEOFF_DURATION_S * TAKEOFF_DURATION_S * TAKEOFF_DURATION_S);

	for (int i = 0; i < 3; i++) {
		const float delta = _takeoff_target_position_ned[i] - _takeoff_start_position_ned[i];
		output.position_ned[i] = _takeoff_start_position_ned[i] + delta * h;
		output.velocity_ned[i] = delta * h_dot;
		output.acceleration_ned[i] = delta * h_ddot;
		output.jerk_ned[i] = delta * h_jerk;
		output.snap_ned[i] = 0.f;
	}
}

void TrajectoryGenerator::update_manual_hold_target(const Input &input, Output &output)
{
	const float target_vz_ned = update_manual_height_reference(input);

	copy3(_manual_hold_position_ned, output.position_ned);
	set_zero_derivatives(output);
	output.velocity_ned[2] = target_vz_ned;
	sync_hover_reference_from_output(output);
}

float TrajectoryGenerator::update_manual_height_reference(const Input &input)
{
	if (!_manual_hold_initialized) {
		copy3(_hover_position_ned, _manual_hold_position_ned);
		_manual_hold_initialized = true;
		_last_update_us = input.timestamp_us;
	}

	const float dt = math::constrain((input.timestamp_us - _last_update_us) * 1e-6f, 0.f, 0.1f);
	float stick = input.manual_height_control_valid ? math::constrain(input.manual_height_stick, -1.f, 1.f) : 0.f;

	// 1/2 command sentinels must never be interpreted as height-stick commands.
	if (fabsf(stick - KEYBOARD_TAKEOFF_COMMAND_STICK) < MANUAL_COMMAND_EPS
	    || fabsf(stick - KEYBOARD_LAND_COMMAND_STICK) < MANUAL_COMMAND_EPS
	    || fabsf(stick) < MANUAL_HEIGHT_DEADZONE) {
		stick = 0.f;
	}

	const float target_vz_ned = -stick * MANUAL_MAX_CLIMB_RATE_M_S;
	_manual_hold_position_ned[2] += target_vz_ned * dt;

	const float min_z_ned = _ground_position_ned[2] - MANUAL_MAX_HEIGHT_M;
	const float max_z_ned = _ground_position_ned[2] - MANUAL_MIN_HEIGHT_M;
	_manual_hold_position_ned[2] = math::constrain(_manual_hold_position_ned[2], min_z_ned, max_z_ned);

	_hover_position_ned[2] = _manual_hold_position_ned[2];
	return target_vz_ned;
}

void TrajectoryGenerator::update_hover_target(const Input &input, Output &output)
{
	output.mode = Mode::Hover;
	output.yaw = _start_yaw;

	if (input.manual_height_control_enabled) {
		update_manual_hold_target(input, output);

	} else {
		_manual_hold_initialized = false;
		set_hold_position(output, _hover_position_ned);
	}
}

void TrajectoryGenerator::update_landing_target(const Input &input, Output &output)
{
	const float elapsed_s = math::max((input.timestamp_us - _landing_start_time_us) * 1e-6f, 0.f);
	output.elapsed_time_s = elapsed_s;
	output.yaw = _landing_yaw;

	if (elapsed_s >= LANDING_DURATION_S) {
		output.mode = Mode::Landed;
		copy3(_landing_target_position_ned, output.position_ned);

		// Bias the final target slightly below the captured ground plane. The
		// vehicle cannot physically follow it through the floor, so the vertical
		// position loop reduces thrust while the PX4 land detector confirms
		// touchdown. The keyboard node then requests a real Commander disarm.
		output.position_ned[2] = _ground_position_ned[2] + LANDED_TARGET_BIAS_M;
		set_zero_derivatives(output);
		return;
	}

	output.mode = Mode::Landing;
	const float s = math::constrain(elapsed_s / LANDING_DURATION_S, 0.f, 1.f);
	const float T = LANDING_DURATION_S;

	// Fifth-order smoothstep. Position, velocity and acceleration are continuous
	// and both velocity and acceleration reach zero at touchdown.
	const float s2 = s * s;
	const float s3 = s2 * s;
	const float s4 = s3 * s;
	const float s5 = s4 * s;
	const float h = 10.f * s3 - 15.f * s4 + 6.f * s5;
	const float h_dot = (30.f * s2 - 60.f * s3 + 30.f * s4) / T;
	const float h_ddot = (60.f * s - 180.f * s2 + 120.f * s3) / (T * T);
	const float h_jerk = (60.f - 360.f * s + 360.f * s2) / (T * T * T);
	const float h_snap = (-360.f + 720.f * s) / (T * T * T * T);

	for (int i = 0; i < 3; i++) {
		const float delta = _landing_target_position_ned[i] - _landing_start_position_ned[i];
		output.position_ned[i] = _landing_start_position_ned[i] + delta * h;
		output.velocity_ned[i] = delta * h_dot;
		output.acceleration_ned[i] = delta * h_ddot;
		output.jerk_ned[i] = delta * h_jerk;
		output.snap_ned[i] = delta * h_snap;
	}
}

void TrajectoryGenerator::sync_hover_reference_from_output(const Output &output)
{
	copy3(output.position_ned, _hover_position_ned);
}
