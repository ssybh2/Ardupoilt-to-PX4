#include "SimplePIDController.hpp"

#include <math.h>

namespace
{

// Keep the DSun gains, but make its rigid-body model consistent with the
// existing shut_m1 L1 predictor and 3 kg SIH test vehicle.
static constexpr float VEHICLE_MASS_KG = 3.0f;
static constexpr float JXX_KGM2 = 0.023f;
static constexpr float JYY_KGM2 = 0.023f;
static constexpr float JZZ_KGM2 = 0.0459f;

}

SimplePIDController::SimplePIDController()
{
	GeometricController::Parameters parameters = _geometric_controller.parameters();
	parameters.mass_kg = VEHICLE_MASS_KG;
	parameters.inertia_kg_m2[0] = JXX_KGM2;
	parameters.inertia_kg_m2[1] = JYY_KGM2;
	parameters.inertia_kg_m2[2] = JZZ_KGM2;
	_geometric_controller.set_parameters(parameters);
}

void SimplePIDController::reset()
{
	_last_input = Input{};
	_last_output = Output{};
}

bool SimplePIDController::update(const Input &input, Output &output)
{
	_last_input = input;
	output = Output{};
	output.timestamp_us = input.timestamp_us;

	GeometricController::Input geometric_input{};
	geometric_input.timestamp_us = input.timestamp_us;

	for (int i = 0; i < 3; i++) {
		geometric_input.position_ned[i] = input.position_ned[i];
		geometric_input.velocity_ned[i] = input.velocity_ned[i];
		geometric_input.angular_velocity_body[i] = input.angular_velocity_body[i];
		geometric_input.target_position_ned[i] = input.target_position_ned[i];
		geometric_input.target_velocity_ned[i] = input.target_velocity_ned[i];
		geometric_input.target_acceleration_ned[i] = input.target_acceleration_ned[i];
		geometric_input.target_jerk_ned[i] = input.target_jerk_ned[i];
		geometric_input.target_snap_ned[i] = input.target_snap_ned[i];
	}

	for (int i = 0; i < 4; i++) {
		geometric_input.quat_body_to_ned[i] = input.quat_body_to_ned[i];
	}

	// Convert scalar yaw trajectory into DSun's heading-vector interface.
	const float yaw = input.target_yaw;
	const float yaw_rate = input.target_yaw_rate;
	const float yaw_accel = input.target_yaw_accel;
	const float c = cosf(yaw);
	const float s = sinf(yaw);

	geometric_input.target_yaw[0] = c;
	geometric_input.target_yaw[1] = s;
	geometric_input.target_yaw_dot[0] = -s * yaw_rate;
	geometric_input.target_yaw_dot[1] = c * yaw_rate;
	geometric_input.target_yaw_ddot[0] = -c * yaw_rate * yaw_rate - s * yaw_accel;
	geometric_input.target_yaw_ddot[1] = -s * yaw_rate * yaw_rate + c * yaw_accel;

	// shut_m1 is specifically the M1-failure spin-hover branch. Absolute yaw
	// control is deliberately released; the allocator likewise removes yaw
	// authority after one motor failure.
	geometric_input.yaw_control_enabled = false;

	geometric_input.state_valid_for_control = input.state_valid_for_control;
	geometric_input.armed = input.armed;
	geometric_input.failsafe = input.failsafe;
	geometric_input.nav_state = input.nav_state;

	GeometricController::Output geometric_output{};
	const bool executed = _geometric_controller.update(geometric_input, geometric_output);

	// Preserve the historical SimplePID output convention for diagnostics.
	// DSun errors are current-target, while this adapter exposed target-current.
	for (int i = 0; i < 3; i++) {
		output.position_error_ned[i] = -geometric_output.r_error[i];
		output.velocity_error_ned[i] = -geometric_output.v_error[i];
		output.moment_newton_meter[i] = geometric_output.M[i];
	}

	output.thrust_newton = geometric_output.target_thrust;
	output.valid = geometric_output.valid;

	if (!output.valid) {
		output.thrust_newton = 0.f;
		output.moment_newton_meter[0] = 0.f;
		output.moment_newton_meter[1] = 0.f;
		output.moment_newton_meter[2] = 0.f;
	}

	_last_output = output;
	return executed;
}
