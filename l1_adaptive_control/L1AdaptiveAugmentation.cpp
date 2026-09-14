#include "L1AdaptiveAugmentation.hpp"

#include <matrix/matrix/math.hpp>

#include <math.h>

using matrix::Dcmf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Vector4f;

namespace
{

static constexpr float GRAVITY_MAGNITUDE = 9.80665f;
static constexpr float SOURCE_DT = 0.0025f;

void copy3(const Vector3f &input, float output[3])
{
	for (int i = 0; i < 3; i++) {
		output[i] = input(i);
	}
}

void copy4(const Vector4f &input, float output[4])
{
	for (int i = 0; i < 4; i++) {
		output[i] = input(i);
	}
}

void copy2(const Vector2f &input, float output[2])
{
	for (int i = 0; i < 2; i++) {
		output[i] = input(i);
	}
}

} // namespace

void L1AdaptiveAugmentation::reset()
{
	_initialized = false;
	_v_hat_prev = Vector3f{};
	_omega_hat_prev = Vector3f{};
	_v_prev = Vector3f{};
	_omega_prev = Vector3f{};
	_R_prev = Dcmf{};
	_u_b_prev = Vector4f{};
	_u_ad_prev = Vector4f{};
	_sigma_m_hat_prev = Vector4f{};
	_sigma_um_hat_prev = Vector2f{};
	_lpf1_prev = Vector4f{};
	_lpf2_prev = Vector4f{};
}

void L1AdaptiveAugmentation::initialize_from_input(const Input &input)
{
	// Source-equivalent ModeAdaptive::init() state.  In particular u_b_prev is
	// zero here; the current baseline is stored only at the end of the first
	// L1AdaptiveAugmentation update.
	_v_hat_prev = Vector3f{input.velocity_ned[0], input.velocity_ned[1], input.velocity_ned[2]};
	_v_prev = _v_hat_prev;
	_omega_hat_prev = Vector3f{input.angular_velocity_body[0], input.angular_velocity_body[1],
				    input.angular_velocity_body[2]};
	_omega_prev = _omega_hat_prev;
	_R_prev = Dcmf{Quatf{input.quat_body_to_ned}};
	_u_b_prev = Vector4f{};
	_u_ad_prev = Vector4f{};
	_sigma_m_hat_prev = Vector4f{};
	_sigma_um_hat_prev = Vector2f{};
	_lpf1_prev = Vector4f{};
	_lpf2_prev = Vector4f{};
	_initialized = true;
}

bool L1AdaptiveAugmentation::update(const Input &input, Output &output)
{
	output = Output{};

	if (!input.baseline_valid || !input.state_valid || !input.armed || input.failsafe) {
		reset();
		return false;
	}

	if (!_initialized) {
		initialize_from_input(input);
	}

	// Direct modular extraction of ModeAdaptive::L1AdaptiveAugmentation().
	Vector3f v_hat;
	Vector3f omega_hat;
	const Vector3f e3{0.f, 0.f, 1.f};
	const float dt = SOURCE_DT;
	const int8_t l1enable = _parameters.l1_enable;

	const Vector3f v_now{input.velocity_ned[0], input.velocity_ned[1], input.velocity_ned[2]};
	const float As_v = _parameters.as_v;
	const float As_omega = _parameters.as_omega;
	const Vector3f omega_now{input.angular_velocity_body[0], input.angular_velocity_body[1],
				 input.angular_velocity_body[2]};
	const float kg_vehicleMass = _parameters.mass_kg;
	const float massInverse = 1.0f / kg_vehicleMass;

	Matrix3f J{};
	J(0, 0) = _parameters.inertia_kg_m2[0];
	J(1, 1) = _parameters.inertia_kg_m2[1];
	J(2, 2) = _parameters.inertia_kg_m2[2];

	Matrix3f Jinv{};
	Jinv(0, 0) = _parameters.inertia_inverse[0];
	Jinv(1, 1) = _parameters.inertia_inverse[1];
	Jinv(2, 2) = _parameters.inertia_inverse[2];

	const Vector3f vpred_error_prev = _v_hat_prev - _v_prev;
	const Vector3f omegapred_error_prev = _omega_hat_prev - _omega_prev;

	v_hat = _v_hat_prev
		+ (e3 * GRAVITY_MAGNITUDE
		   - _R_prev.col(2) * (_u_b_prev(0) + _u_ad_prev(0) + _sigma_m_hat_prev(0)) * massInverse
		   + _R_prev.col(0) * _sigma_um_hat_prev(0) * massInverse
		   + _R_prev.col(1) * _sigma_um_hat_prev(1) * massInverse
		   + vpred_error_prev * As_v) * dt;

	const Vector3f tempVec{
		_u_b_prev(1) + _u_ad_prev(1) + _sigma_m_hat_prev(1),
		_u_b_prev(2) + _u_ad_prev(2) + _sigma_m_hat_prev(2),
		_u_b_prev(3) + _u_ad_prev(3) + _sigma_m_hat_prev(3)
	};

	omega_hat = _omega_hat_prev
		+ (-(Jinv * _omega_prev.cross(J * _omega_prev))
		   + Jinv * tempVec
		   + omegapred_error_prev * As_omega) * dt;

	// Same storage order as the source implementation.
	_v_hat_prev = v_hat;
	_omega_hat_prev = omega_hat;

	const Vector3f vpred_error = v_hat - v_now;
	const Vector3f omegapred_error = omega_hat - omega_now;

	const float exp_As_v_dt = expf(As_v * dt);
	const float exp_As_omega_dt = expf(As_omega * dt);

	const Vector3f PhiInvmu_v = vpred_error / (exp_As_v_dt - 1.f) * As_v * exp_As_v_dt;
	const Vector3f PhiInvmu_omega = omegapred_error / (exp_As_omega_dt - 1.f) * As_omega * exp_As_omega_dt;

	Vector4f sigma_m_hat{};
	Vector2f sigma_um_hat{};
	const Dcmf R{Quatf{input.quat_body_to_ned}};

	sigma_m_hat(0) = R.col(2).dot(PhiInvmu_v) * kg_vehicleMass;
	const Vector3f sigma_m_hat_2to4 = -(J * PhiInvmu_omega);
	sigma_m_hat(1) = sigma_m_hat_2to4(0);
	sigma_m_hat(2) = sigma_m_hat_2to4(1);
	sigma_m_hat(3) = sigma_m_hat_2to4(2);

	sigma_um_hat(0) = -R.col(0).dot(PhiInvmu_v) * kg_vehicleMass;
	sigma_um_hat(1) = -R.col(1).dot(PhiInvmu_v) * kg_vehicleMass;

	_sigma_m_hat_prev = sigma_m_hat;
	_sigma_um_hat_prev = sigma_um_hat;

	const float lpf1_coefficientThrust1 = expf(-_parameters.cutoff_q1_thrust * 0.0025f);
	const float lpf1_coefficientThrust2 = 1.0f - lpf1_coefficientThrust1;
	const float lpf1_coefficientMoment1 = expf(-_parameters.cutoff_q1_moment * 0.0025f);
	const float lpf1_coefficientMoment2 = 1.0f - lpf1_coefficientMoment1;

	Vector4f u_ad_int{};
	Vector4f u_ad{};

	u_ad_int(0) = lpf1_coefficientThrust1 * _lpf1_prev(0) + lpf1_coefficientThrust2 * sigma_m_hat(0);
	u_ad_int(1) = lpf1_coefficientMoment1 * _lpf1_prev(1) + lpf1_coefficientMoment2 * sigma_m_hat(1);
	u_ad_int(2) = lpf1_coefficientMoment1 * _lpf1_prev(2) + lpf1_coefficientMoment2 * sigma_m_hat(2);
	u_ad_int(3) = lpf1_coefficientMoment1 * _lpf1_prev(3) + lpf1_coefficientMoment2 * sigma_m_hat(3);

	_lpf1_prev = u_ad_int;

	const float lpf2_coefficientMoment1 = expf(-_parameters.cutoff_q2_moment * 0.0025f);
	const float lpf2_coefficientMoment2 = 1.0f - lpf2_coefficientMoment1;

	u_ad(0) = u_ad_int(0);
	u_ad(1) = lpf2_coefficientMoment1 * _lpf2_prev(1) + lpf2_coefficientMoment2 * u_ad_int(1);
	u_ad(2) = lpf2_coefficientMoment1 * _lpf2_prev(2) + lpf2_coefficientMoment2 * u_ad_int(2);
	u_ad(3) = lpf2_coefficientMoment1 * _lpf2_prev(3) + lpf2_coefficientMoment2 * u_ad_int(3);

	_lpf2_prev = u_ad;
	u_ad = -u_ad;
	_u_ad_prev = u_ad * static_cast<float>(l1enable);

	_v_prev = v_now;
	_omega_prev = omega_now;
	_R_prev = R;

	Vector4f thrustMomentCmd{};

	for (int i = 0; i < 4; i++) {
		thrustMomentCmd(i) = input.baseline_thrust_moment[i];
	}

	_u_b_prev = thrustMomentCmd;

	copy4(_u_ad_prev, output.adaptive_thrust_moment);
	copy3(v_hat, output.velocity_hat);
	copy3(omega_hat, output.angular_velocity_hat);
	copy4(sigma_m_hat, output.sigma_matched);
	copy2(sigma_um_hat, output.sigma_unmatched);
	copy4(u_ad_int, output.lpf1);
	copy4(_lpf2_prev, output.lpf2);
	output.valid = true;
	return true;
}
