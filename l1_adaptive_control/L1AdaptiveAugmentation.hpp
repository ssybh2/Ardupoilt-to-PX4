#pragma once

#include <drivers/drv_hrt.h>

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

		float velocity_hat_prev[3]{0.f, 0.f, 0.f};
		float angular_velocity_hat_prev[3]{0.f, 0.f, 0.f};
		float velocity_prev[3]{0.f, 0.f, 0.f};
		float angular_velocity_prev[3]{0.f, 0.f, 0.f};
		float rotation_body_to_ned_prev[3][3]{{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}};

		float baseline_thrust_moment_prev[4]{0.f, 0.f, 0.f, 0.f};
		float adaptive_thrust_moment_prev[4]{0.f, 0.f, 0.f, 0.f};
		float sigma_matched_prev[4]{0.f, 0.f, 0.f, 0.f};
		float sigma_unmatched_prev[2]{0.f, 0.f};
		float lpf1_prev[4]{0.f, 0.f, 0.f, 0.f};
		float lpf2_prev[4]{0.f, 0.f, 0.f, 0.f};
	};

	State _state{};
};
