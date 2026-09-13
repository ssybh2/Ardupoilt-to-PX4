#include "L1AdaptiveAugmentation.hpp"

#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using matrix::Dcmf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Vector4f;

namespace
{

static constexpr float VEHICLE_MASS_KG = 3.0f;
static constexpr float GRAVITY_MSS = 9.80665f;

static constexpr float JXX_KGM2 = 0.023f;
static constexpr float JYY_KGM2 = 0.023f;
static constexpr float JZZ_KGM2 = 0.0459f;

static constexpr float JINV_XX = 43.478f;
static constexpr float JINV_YY = 43.478f;
static constexpr float JINV_ZZ = 21.786f;

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

float phi_inverse_mu(float prediction_error, float as_value, float dt)
{
	const float exp_as_dt = expf(as_value * dt);
	const float denominator = exp_as_dt - 1.0f;

	if (fabsf(denominator) < 1e-5f || fabsf(as_value) < 1e-5f) {
		return 0.f;
	}

	return prediction_error / denominator * as_value * exp_as_dt;
}

} // namespace

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

	const Vector3f velocity_now{input.velocity_ned[0], input.velocity_ned[1], input.velocity_ned[2]};
	const Vector3f omega_now{input.angular_velocity_body[0], input.angular_velocity_body[1],
				 input.angular_velocity_body[2]};
	Vector4f baseline_thrust_moment{};

	for (int i = 0; i < 4; i++) {
		baseline_thrust_moment(i) = input.baseline_thrust_moment[i];
	}

	const Matrix3f rotation{Dcmf{Quatf{input.quat_body_to_ned}}};
	const Vector3f body_x{rotation.col(0)};
	const Vector3f body_y{rotation.col(1)};
	const Vector3f body_z{rotation.col(2)};
	const Vector3f e3{0.f, 0.f, 1.f};

	Matrix3f inertia{};
	inertia(0, 0) = JXX_KGM2;
	inertia(1, 1) = JYY_KGM2;
	inertia(2, 2) = JZZ_KGM2;

	Matrix3f inertia_inverse{};
	inertia_inverse(0, 0) = JINV_XX;
	inertia_inverse(1, 1) = JINV_YY;
	inertia_inverse(2, 2) = JINV_ZZ;

	if (!_state.initialized) {
		_state.velocity_hat_prev = velocity_now;
		_state.velocity_prev = velocity_now;
		_state.angular_velocity_hat_prev = omega_now;
		_state.angular_velocity_prev = omega_now;
		_state.rotation_body_to_ned_prev = rotation;
		_state.baseline_thrust_moment_prev = baseline_thrust_moment;
		_state.last_update_us = input.timestamp_us;
		_state.initialized = true;
	}

	const float dt = math::constrain((input.timestamp_us - _state.last_update_us) * 1e-6f, 0.001f, 0.02f);

	const Vector3f velocity_prediction_error_prev = _state.velocity_hat_prev - _state.velocity_prev;
	const Vector3f angular_prediction_error_prev =
		_state.angular_velocity_hat_prev - _state.angular_velocity_prev;

	const float previous_total_thrust =
		_state.baseline_thrust_moment_prev(0)
		+ _state.adaptive_thrust_moment_prev(0)
		+ _state.sigma_matched_prev(0);

	const Vector3f body_x_prev{_state.rotation_body_to_ned_prev.col(0)};
	const Vector3f body_y_prev{_state.rotation_body_to_ned_prev.col(1)};
	const Vector3f body_z_prev{_state.rotation_body_to_ned_prev.col(2)};

	const Vector3f velocity_hat =
		_state.velocity_hat_prev
		+ (e3 * GRAVITY_MSS
		   - body_z_prev * (previous_total_thrust / VEHICLE_MASS_KG)
		   + body_x_prev * (_state.sigma_unmatched_prev(0) / VEHICLE_MASS_KG)
		   + body_y_prev * (_state.sigma_unmatched_prev(1) / VEHICLE_MASS_KG)
		   + velocity_prediction_error_prev * L1_AS_V) * dt;

	const Vector3f previous_total_moment{
		_state.baseline_thrust_moment_prev(1) + _state.adaptive_thrust_moment_prev(1) + _state.sigma_matched_prev(1),
		_state.baseline_thrust_moment_prev(2) + _state.adaptive_thrust_moment_prev(2) + _state.sigma_matched_prev(2),
		_state.baseline_thrust_moment_prev(3) + _state.adaptive_thrust_moment_prev(3) + _state.sigma_matched_prev(3)
	};

	const Vector3f gyro_moment_prev =
		_state.angular_velocity_prev.cross(inertia * _state.angular_velocity_prev);

	const Vector3f angular_velocity_hat =
		_state.angular_velocity_hat_prev
		+ (-(inertia_inverse * gyro_moment_prev)
		   + inertia_inverse * previous_total_moment
		   + angular_prediction_error_prev * L1_AS_OMEGA) * dt;

	const Vector3f velocity_prediction_error = velocity_hat - velocity_now;
	const Vector3f angular_prediction_error = angular_velocity_hat - omega_now;

	Vector3f phi_inv_mu_v{};
	Vector3f phi_inv_mu_omega{};

	for (int i = 0; i < 3; i++) {
		phi_inv_mu_v(i) = phi_inverse_mu(velocity_prediction_error(i), L1_AS_V, dt);
		phi_inv_mu_omega(i) = phi_inverse_mu(angular_prediction_error(i), L1_AS_OMEGA, dt);
	}

	Vector4f sigma_matched{};
	Vector2f sigma_unmatched{};

	sigma_matched(0) = body_z.dot(phi_inv_mu_v) * VEHICLE_MASS_KG;
	const Vector3f sigma_matched_moment = -(inertia * phi_inv_mu_omega);

	for (int i = 0; i < 3; i++) {
		sigma_matched(i + 1) = sigma_matched_moment(i);
	}

	sigma_unmatched(0) = -body_x.dot(phi_inv_mu_v) * VEHICLE_MASS_KG;
	sigma_unmatched(1) = -body_y.dot(phi_inv_mu_v) * VEHICLE_MASS_KG;

	const float lpf2_moment_keep = expf(-L1_CUTOFF_Q2_MOMENT * dt);
	Vector4f lpf1{};
	Vector4f lpf2{};

	for (int i = 0; i < 4; i++) {
		const float cutoff_q1 = (i == 0) ? L1_CUTOFF_Q1_THRUST : L1_CUTOFF_Q1_MOMENT;
		const float lpf1_keep = expf(-cutoff_q1 * dt);
		lpf1(i) = lpf1_keep * _state.lpf1_prev(i) + (1.f - lpf1_keep) * sigma_matched(i);

		if (i == 0) {
			lpf2(i) = lpf1(i);

		} else {
			lpf2(i) = lpf2_moment_keep * _state.lpf2_prev(i) + (1.f - lpf2_moment_keep) * lpf1(i);
		}

		const float adaptive_command = L1_ENABLE
					       ? math::constrain(-lpf2(i), -ADAPTIVE_LIMITS[i], ADAPTIVE_LIMITS[i])
					       : 0.f;
		output.adaptive_thrust_moment[i] = adaptive_command;
		output.combined_thrust_moment[i] = baseline_thrust_moment(i) + adaptive_command;
	}

	for (int i = 0; i < 2; i++) {
		_state.sigma_unmatched_prev(i) =
			math::constrain(sigma_unmatched(i), -MAX_L1_THRUST_N, MAX_L1_THRUST_N);
	}

	_state.velocity_hat_prev = velocity_hat;
	_state.velocity_prev = velocity_now;
	_state.angular_velocity_hat_prev = angular_velocity_hat;
	_state.angular_velocity_prev = omega_now;
	_state.rotation_body_to_ned_prev = rotation;
	_state.baseline_thrust_moment_prev = baseline_thrust_moment;
	_state.sigma_matched_prev = sigma_matched;
	_state.lpf1_prev = lpf1;
	_state.lpf2_prev = lpf2;

	for (int i = 0; i < 4; i++) {
		_state.adaptive_thrust_moment_prev(i) = output.adaptive_thrust_moment[i];
	}

	_state.last_update_us = input.timestamp_us;
	output.valid = true;
	return true;
}
