#include "SimplePIDController.hpp"

#include <math.h>

namespace
{

static constexpr float VEHICLE_MASS_KG = 3.0f;
static constexpr float GRAVITY_MSS = 9.80665f;
static constexpr float PI_F = 3.14159265358979323846f;

// Keep the outer-loop behavior close to the existing geometric controller.
// The old force gains divided by mass are used as acceleration gains.
static constexpr float POS_KP_X = 18.0f / VEHICLE_MASS_KG;
static constexpr float POS_KP_Y = 18.0f / VEHICLE_MASS_KG;
static constexpr float POS_KP_Z = 27.6f / VEHICLE_MASS_KG;
static constexpr float VEL_KD_X = 4.0f / VEHICLE_MASS_KG;
static constexpr float VEL_KD_Y = 4.0f / VEHICLE_MASS_KG;
static constexpr float VEL_KD_Z = 6.0f / VEHICLE_MASS_KG;

// Simple Euler attitude PID gains. These are deliberately close to the old
// geometric PD gains so the first SITL comparison starts from a similar scale.
static constexpr float ATT_KP[3] = {5.4f, 5.4f, 0.092f};
static constexpr float ATT_KI[3] = {0.15f, 0.15f, 0.01f};
static constexpr float ATT_KD[3] = {0.6f, 0.6f, 0.023f};

static constexpr float ATT_INT_LIMIT[3] = {0.35f, 0.35f, 0.50f};
static constexpr float MAX_TILT_RAD = 35.0f * PI_F / 180.0f;
static constexpr float MAX_HORIZONTAL_ACCEL_MSS = 4.0f;
static constexpr float MAX_VERTICAL_ACCEL_MSS = 5.0f;
static constexpr float MAX_THRUST_N = VEHICLE_MASS_KG * GRAVITY_MSS * 2.0f;

float clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}

	if (value > max_value) {
		return max_value;
	}

	return value;
}

float wrap_pi(float angle)
{
	while (angle > PI_F) {
		angle -= 2.0f * PI_F;
	}

	while (angle < -PI_F) {
		angle += 2.0f * PI_F;
	}

	return angle;
}

bool finite3(const float v[3])
{
	return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

void quat_to_euler_body_to_ned(const float q[4], float euler_rpy[3])
{
	const float w = q[0];
	const float x = q[1];
	const float y = q[2];
	const float z = q[3];

	const float sinr_cosp = 2.0f * (w * x + y * z);
	const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	euler_rpy[0] = atan2f(sinr_cosp, cosr_cosp);

	const float sinp = clampf(2.0f * (w * y - z * x), -1.0f, 1.0f);
	euler_rpy[1] = asinf(sinp);

	const float siny_cosp = 2.0f * (w * z + x * y);
	const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
	euler_rpy[2] = atan2f(siny_cosp, cosy_cosp);
}

} // namespace

void SimplePIDController::reset()
{
	_attitude_integral_rpy[0] = 0.f;
	_attitude_integral_rpy[1] = 0.f;
	_attitude_integral_rpy[2] = 0.f;
	_last_update_us = 0;
	_was_active = false;
}

bool SimplePIDController::update(const Input &input, Output &output)
{
	_last_input = input;
	output = Output{};
	output.timestamp_us = input.timestamp_us;

	if (!input.state_valid_for_control || !input.armed || input.failsafe) {
		reset();
		_last_output = output;
		return true;
	}

	float dt = 0.004f;

	if (_last_update_us != 0 && input.timestamp_us > _last_update_us) {
		dt = clampf((input.timestamp_us - _last_update_us) * 1e-6f, 0.001f, 0.02f);
	}

	_last_update_us = input.timestamp_us;

	if (!_was_active) {
		_attitude_integral_rpy[0] = 0.f;
		_attitude_integral_rpy[1] = 0.f;
		_attitude_integral_rpy[2] = 0.f;
		_was_active = true;
	}

	for (int i = 0; i < 3; i++) {
		output.position_error_ned[i] = input.target_position_ned[i] - input.position_ned[i];
		output.velocity_error_ned[i] = input.target_velocity_ned[i] - input.velocity_ned[i];
	}

	output.desired_acceleration_ned[0] = input.target_acceleration_ned[0]
		+ POS_KP_X * output.position_error_ned[0]
		+ VEL_KD_X * output.velocity_error_ned[0];
	output.desired_acceleration_ned[1] = input.target_acceleration_ned[1]
		+ POS_KP_Y * output.position_error_ned[1]
		+ VEL_KD_Y * output.velocity_error_ned[1];
	output.desired_acceleration_ned[2] = input.target_acceleration_ned[2]
		+ POS_KP_Z * output.position_error_ned[2]
		+ VEL_KD_Z * output.velocity_error_ned[2];

	output.desired_acceleration_ned[0] = clampf(output.desired_acceleration_ned[0],
		-MAX_HORIZONTAL_ACCEL_MSS, MAX_HORIZONTAL_ACCEL_MSS);
	output.desired_acceleration_ned[1] = clampf(output.desired_acceleration_ned[1],
		-MAX_HORIZONTAL_ACCEL_MSS, MAX_HORIZONTAL_ACCEL_MSS);
	output.desired_acceleration_ned[2] = clampf(output.desired_acceleration_ned[2],
		-MAX_VERTICAL_ACCEL_MSS, MAX_VERTICAL_ACCEL_MSS);

	quat_to_euler_body_to_ned(input.quat_body_to_ned, output.current_euler_rpy);

	const float yaw_sp = wrap_pi(input.target_yaw);
	const float cy = cosf(yaw_sp);
	const float sy = sinf(yaw_sp);
	const float a_n = output.desired_acceleration_ned[0];
	const float a_e = output.desired_acceleration_ned[1];

	// Small-angle NED/FRD mapping around hover:
	//   roll_sp  ~= ( cos(yaw) * a_e - sin(yaw) * a_n ) / g
	//   pitch_sp ~= -(cos(yaw) * a_n + sin(yaw) * a_e ) / g
	output.desired_euler_rpy[0] = clampf((cy * a_e - sy * a_n) / GRAVITY_MSS,
		-MAX_TILT_RAD, MAX_TILT_RAD);
	output.desired_euler_rpy[1] = clampf(-(cy * a_n + sy * a_e) / GRAVITY_MSS,
		-MAX_TILT_RAD, MAX_TILT_RAD);
	output.desired_euler_rpy[2] = yaw_sp;

	output.attitude_error_rpy[0] = output.desired_euler_rpy[0] - output.current_euler_rpy[0];
	output.attitude_error_rpy[1] = output.desired_euler_rpy[1] - output.current_euler_rpy[1];
	output.attitude_error_rpy[2] = wrap_pi(output.desired_euler_rpy[2] - output.current_euler_rpy[2]);

	const float desired_rate_body[3] = {0.f, 0.f, input.target_yaw_rate};

	for (int i = 0; i < 3; i++) {
		output.rate_error_body[i] = desired_rate_body[i] - input.angular_velocity_body[i];

		_attitude_integral_rpy[i] += output.attitude_error_rpy[i] * dt;
		_attitude_integral_rpy[i] = clampf(_attitude_integral_rpy[i],
			-ATT_INT_LIMIT[i], ATT_INT_LIMIT[i]);
		output.attitude_integral_rpy[i] = _attitude_integral_rpy[i];

		output.moment_newton_meter[i] = ATT_KP[i] * output.attitude_error_rpy[i]
			+ ATT_KI[i] * output.attitude_integral_rpy[i]
			+ ATT_KD[i] * output.rate_error_body[i];
	}

	// NED z acceleration is positive downward. Around hover:
	//   a_z = g - T/m, so T = m * (g - a_z).
	// Compensate for the commanded tilt so vertical thrust remains comparable.
	float tilt_projection = cosf(output.desired_euler_rpy[0]) * cosf(output.desired_euler_rpy[1]);
	tilt_projection = clampf(tilt_projection, 0.5f, 1.0f);

	const float vertical_specific_force = GRAVITY_MSS - output.desired_acceleration_ned[2];
	output.thrust_newton = VEHICLE_MASS_KG * vertical_specific_force / tilt_projection;
	output.thrust_newton = clampf(output.thrust_newton, 0.f, MAX_THRUST_N);

	output.valid = isfinite(output.thrust_newton)
		&& output.thrust_newton > 0.f
		&& finite3(output.position_error_ned)
		&& finite3(output.velocity_error_ned)
		&& finite3(output.desired_acceleration_ned)
		&& finite3(output.current_euler_rpy)
		&& finite3(output.desired_euler_rpy)
		&& finite3(output.attitude_error_rpy)
		&& finite3(output.attitude_integral_rpy)
		&& finite3(output.rate_error_body)
		&& finite3(output.moment_newton_meter);

	if (!output.valid) {
		output.thrust_newton = 0.f;
		output.moment_newton_meter[0] = 0.f;
		output.moment_newton_meter[1] = 0.f;
		output.moment_newton_meter[2] = 0.f;
		reset();
	}

	_last_output = output;
	return true;
}
