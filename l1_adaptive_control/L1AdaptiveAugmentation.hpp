#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

class L1AdaptiveAugmentation
{
public:
	struct Input {
		hrt_abstime timestamp_us{0};
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
		float combined_thrust_moment[4]{0.f, 0.f, 0.f, 0.f};
		bool valid{false};
	};

	bool update(const Input &input, Output &output);
	void reset();

private:
	struct State {
		bool initialized{false};
		hrt_abstime last_update_us{0};

		matrix::Vector3f velocity_hat_prev{};
		matrix::Vector3f angular_velocity_hat_prev{};
		matrix::Vector3f velocity_prev{};
		matrix::Vector3f angular_velocity_prev{};
		matrix::Matrix3f rotation_body_to_ned_prev{};

		matrix::Vector4f baseline_thrust_moment_prev{};
		matrix::Vector4f adaptive_thrust_moment_prev{};
		matrix::Vector4f sigma_matched_prev{};
		matrix::Vector2f sigma_unmatched_prev{};
		matrix::Vector4f lpf1_prev{};
		matrix::Vector4f lpf2_prev{};
	};

	State _state{};
};
