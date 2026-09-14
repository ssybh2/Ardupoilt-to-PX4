#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

#include <stdint.h>

class L1AdaptiveAugmentation
{
public:
	struct Parameters {
		// REAL_OR_SITL source defaults (real-aircraft branch / uploaded DSun set).
		// The orchestrator explicitly loads the SITL values for SITL builds.
		float mass_kg{0.62f};
		float inertia_kg_m2[3]{0.002016f, 0.001827f, 0.00322f};
		float inertia_inverse[3]{496.03f, 547.345f, 310.559f};
		float as_v{-5.f};
		float as_omega{-10.f};
		float cutoff_q1_thrust{30.f};
		float cutoff_q1_moment{5.f};
		float cutoff_q2_moment{15.f};
		int8_t l1_enable{0};
	};

	struct Input {
		hrt_abstime timestamp_us{0}; // PX4 interface only; source equations use fixed dt.
		float velocity_ned[3]{0.f, 0.f, 0.f};
		float quat_body_to_ned[4]{1.f, 0.f, 0.f, 0.f};
		float angular_velocity_body[3]{0.f, 0.f, 0.f};
		float baseline_thrust_moment[4]{0.f, 0.f, 0.f, 0.f};
		bool baseline_valid{false};
		bool state_valid{false};
		bool armed{false};
		bool failsafe{false};
	};

	struct Output {
		float adaptive_thrust_moment[4]{0.f, 0.f, 0.f, 0.f};
		float velocity_hat[3]{0.f, 0.f, 0.f};
		float angular_velocity_hat[3]{0.f, 0.f, 0.f};
		float sigma_matched[4]{0.f, 0.f, 0.f, 0.f};
		float sigma_unmatched[2]{0.f, 0.f};
		float lpf1[4]{0.f, 0.f, 0.f, 0.f};
		float lpf2[4]{0.f, 0.f, 0.f, 0.f};
		bool valid{false};
	};

	bool update(const Input &input, Output &output);
	void reset();
	void set_parameters(const Parameters &parameters) { _parameters = parameters; }

	const Parameters &parameters() const { return _parameters; }

private:
	void initialize_from_input(const Input &input);

	Parameters _parameters{};
	bool _initialized{false};

	matrix::Vector3f _v_hat_prev{};
	matrix::Vector3f _omega_hat_prev{};
	matrix::Vector3f _v_prev{};
	matrix::Vector3f _omega_prev{};
	matrix::Dcmf _R_prev{};
	matrix::Vector4f _u_b_prev{};
	matrix::Vector4f _u_ad_prev{};
	matrix::Vector4f _sigma_m_hat_prev{};
	matrix::Vector2f _sigma_um_hat_prev{};
	matrix::Vector4f _lpf1_prev{};
	matrix::Vector4f _lpf2_prev{};
};
