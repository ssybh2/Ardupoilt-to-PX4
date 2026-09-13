#include "L1AdaptiveAugmentation.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

namespace
{

static constexpr float VEHICLE_MASS_KG = 3.0f;
static constexpr float GRAVITY_MSS = 9.80665f;

static constexpr float INERTIA_KGM2[3] = {0.023f, 0.023f, 0.0459f};
static constexpr float INERTIA_INVERSE[3] = {43.478f, 43.478f, 21.786f};

static constexpr bool L1_ENABLE = true;
static constexpr float L1_AS_V = -5.0f;
static constexpr float L1_AS_OMEGA = -10.0f;
static constexpr float L1_CUTOFF_Q1_THRUST = 10.0f;
static constexpr float L1_CUTOFF_Q1_MOMENT = 10.0f;
static constexpr float L1_CUTOFF_Q2_MOMENT = 2.0f;

static constexpr float MAX_L1_THRUST_N = VEHICLE_MASS_KG * GRAVITY_MSS * 0.35f;
static constexpr float MAX_L1_ROLL_PITCH_MOMENT_NM = 0.35f;
static constexpr float MAX_L1_YAW_MOMENT_NM = 0.20f;
static constexpr float ADAPTIVE_LIMITS[4] = {
	MAX_L1_THRUST_N,
	MAX_L1_ROLL_PITCH_MOMENT_NM,
	MAX_L1_ROLL_PITCH_MOMENT_NM,
	MAX_L1_YAW_MOMENT_NM
};

void quat_to_rotation_matrix_body_to_ned(const float q[4], float rotation[3][3])
{
	const float w = q[0];
	const float x = q[1];
	const float y = q[2];
	const float z = q[3];

	rotation[0][0] = 1.0f - 2.0f * (y * y + z * z);
	rotation[0][1] = 2.0f * (x * y - w * z);
	rotation[0][2] = 2.0f * (x * z + w * y);

	rotation[1][0] = 2.0f * (x * y + w * z);
	rotation[1][1] = 1.0f - 2.0f * (x * x + z * z);
	rotation[1][2] = 2.0f * (y * z - w * x);

	rotation[2][0] = 2.0f * (x * z - w * y);
	rotation[2][1] = 2.0f * (y * z + w * x);
	rotation[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

void cross3(const float a[3], const float b[3], float output[3])
{
	for (int i = 0; i < 3; i++) {
		const int j = (i + 1) % 3;
		const int k = (i + 2) % 3;
		output[i] = a[j] * b[k] - a[k] * b[j];
	}
}

float dot_matrix_column(const float matrix[3][3], int column, const float vector[3])
{
	float result = 0.f;

	for (int i = 0; i < 3; i++) {
		result += matrix[i][column] * vector[i];
	}

	return result;
}

float phi_inverse_mu(float prediction_error, float as_value, float dt)
{
	const float exp_as_dt = expf(as_value * dt);
	const float denominator = exp_as_dt - 1.0f;

	if (fabsf(denominator) < 1e-5f || fabsf(as_value) < 1e-5f) {
		return 0.f;
	}

	return prediction_error / denominator * as_value * exp_as_dt;
}

}

void L1AdaptiveAugmentation::reset()
{
	_state = State{};
}

bool L1AdaptiveAugmentation::update(const Input &input, Output &output)
{
	output = Output{};

	if (!input.baseline_valid || !input.state_valid || !input.armed || input.failsafe) {
		reset();
		return false;
	}

	float rotation[3][3]{};
	quat_to_rotation_matrix_body_to_ned(input.quat_body_to_ned, rotation);

	if (!_state.initialized) {
		for (int i = 0; i < 3; i++) {
			_state.velocity_hat_prev[i] = input.velocity_ned[i];
			_state.velocity_prev[i] = input.velocity_ned[i];
			_state.angular_velocity_hat_prev[i] = input.angular_velocity_body[i];
			_state.angular_velocity_prev[i] = input.angular_velocity_body[i];
		}

		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				_state.rotation_body_to_ned_prev[row][col] = rotation[row][col];
			}
		}

		for (int i = 0; i < 4; i++) {
			_state.baseline_thrust_moment_prev[i] = input.baseline_thrust_moment[i];
		}

		_state.last_update_us = input.timestamp_us;
		_state.initialized = true;
	}

	const float dt = math::constrain((input.timestamp_us - _state.last_update_us) * 1e-6f, 0.001f, 0.02f);

	float velocity_prediction_error_prev[3]{};
	float angular_prediction_error_prev[3]{};

	for (int i = 0; i < 3; i++) {
		velocity_prediction_error_prev[i] = _state.velocity_hat_prev[i] - _state.velocity_prev[i];
		angular_prediction_error_prev[i] = _state.angular_velocity_hat_prev[i] - _state.angular_velocity_prev[i];
	}

	const float previous_total_thrust =
		_state.baseline_thrust_moment_prev[0]
		+ _state.adaptive_thrust_moment_prev[0]
		+ _state.sigma_matched_prev[0];

	float velocity_hat[3]{};

	for (int i = 0; i < 3; i++) {
		const float gravity_term = (i == 2) ? GRAVITY_MSS : 0.f;

		velocity_hat[i] = _state.velocity_hat_prev[i]
				  + (gravity_term
				     - _state.rotation_body_to_ned_prev[i][2] * previous_total_thrust / VEHICLE_MASS_KG
				     + _state.rotation_body_to_ned_prev[i][0] * _state.sigma_unmatched_prev[0] / VEHICLE_MASS_KG
				     + _state.rotation_body_to_ned_prev[i][1] * _state.sigma_unmatched_prev[1] / VEHICLE_MASS_KG
				     + velocity_prediction_error_prev[i] * L1_AS_V) * dt;
	}

	float inertia_omega_prev[3]{};

	for (int i = 0; i < 3; i++) {
		inertia_omega_prev[i] = INERTIA_KGM2[i] * _state.angular_velocity_prev[i];
	}

	float gyro_moment_prev[3]{};
	cross3(_state.angular_velocity_prev, inertia_omega_prev, gyro_moment_prev);

	float previous_total_moment[3]{};

	for (int i = 0; i < 3; i++) {
		const int channel = i + 1;
		previous_total_moment[i] = _state.baseline_thrust_moment_prev[channel]
					   + _state.adaptive_thrust_moment_prev[channel]
					   + _state.sigma_matched_prev[channel];
	}

	float angular_velocity_hat[3]{};

	for (int i = 0; i < 3; i++) {
		angular_velocity_hat[i] = _state.angular_velocity_hat_prev[i]
					  + (-INERTIA_INVERSE[i] * gyro_moment_prev[i]
					     + INERTIA_INVERSE[i] * previous_total_moment[i]
					     + angular_prediction_error_prev[i] * L1_AS_OMEGA) * dt;
	}

	float phi_inv_mu_v[3]{};
	float phi_inv_mu_omega[3]{};

	for (int i = 0; i < 3; i++) {
		const float velocity_prediction_error = velocity_hat[i] - input.velocity_ned[i];
		const float angular_prediction_error = angular_velocity_hat[i] - input.angular_velocity_body[i];
		phi_inv_mu_v[i] = phi_inverse_mu(velocity_prediction_error, L1_AS_V, dt);
		phi_inv_mu_omega[i] = phi_inverse_mu(angular_prediction_error, L1_AS_OMEGA, dt);
	}

	float sigma_matched[4]{};
	float sigma_unmatched[2]{};

	sigma_matched[0] = dot_matrix_column(rotation, 2, phi_inv_mu_v) * VEHICLE_MASS_KG;

	for (int i = 0; i < 3; i++) {
		sigma_matched[i + 1] = -INERTIA_KGM2[i] * phi_inv_mu_omega[i];
	}

	for (int i = 0; i < 2; i++) {
		sigma_unmatched[i] = -dot_matrix_column(rotation, i, phi_inv_mu_v) * VEHICLE_MASS_KG;
	}

	const float lpf2_moment_keep = expf(-L1_CUTOFF_Q2_MOMENT * dt);
	float lpf1[4]{};
	float lpf2[4]{};

	for (int i = 0; i < 4; i++) {
		const float cutoff_q1 = (i == 0) ? L1_CUTOFF_Q1_THRUST : L1_CUTOFF_Q1_MOMENT;
		const float lpf1_keep = expf(-cutoff_q1 * dt);
		lpf1[i] = lpf1_keep * _state.lpf1_prev[i] + (1.f - lpf1_keep) * sigma_matched[i];

		if (i == 0) {
			lpf2[i] = lpf1[i];

		} else {
			lpf2[i] = lpf2_moment_keep * _state.lpf2_prev[i] + (1.f - lpf2_moment_keep) * lpf1[i];
		}

		output.adaptive_thrust_moment[i] = L1_ENABLE
			? math::constrain(-lpf2[i], -ADAPTIVE_LIMITS[i], ADAPTIVE_LIMITS[i])
			: 0.f;
		output.combined_thrust_moment[i] = input.baseline_thrust_moment[i] + output.adaptive_thrust_moment[i];
	}

	for (int i = 0; i < 2; i++) {
		sigma_unmatched[i] = math::constrain(sigma_unmatched[i], -MAX_L1_THRUST_N, MAX_L1_THRUST_N);
		_state.sigma_unmatched_prev[i] = sigma_unmatched[i];
	}

	for (int i = 0; i < 3; i++) {
		_state.velocity_hat_prev[i] = velocity_hat[i];
		_state.velocity_prev[i] = input.velocity_ned[i];
		_state.angular_velocity_hat_prev[i] = angular_velocity_hat[i];
		_state.angular_velocity_prev[i] = input.angular_velocity_body[i];
	}

	for (int i = 0; i < 4; i++) {
		_state.baseline_thrust_moment_prev[i] = input.baseline_thrust_moment[i];
		_state.adaptive_thrust_moment_prev[i] = output.adaptive_thrust_moment[i];
		_state.sigma_matched_prev[i] = sigma_matched[i];
		_state.lpf1_prev[i] = lpf1[i];
		_state.lpf2_prev[i] = lpf2[i];
	}

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			_state.rotation_body_to_ned_prev[row][col] = rotation[row][col];
		}
	}

	_state.last_update_us = input.timestamp_us;
	output.valid = true;
	return true;
}
