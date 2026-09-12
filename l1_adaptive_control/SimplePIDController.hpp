#pragma once

#include "GeometricController.hpp"

#include <drivers/drv_hrt.h>

#include <stdint.h>

// Compatibility adapter retained so the shut_m1 L1 wrapper does not need to
// change its public baseline-controller interface. Internally this class now
// runs the DSun geometric controller.
class SimplePIDController
{
public:
	struct Input {
		hrt_abstime timestamp_us{0};

		float position_ned[3]{0.f, 0.f, 0.f};
		float velocity_ned[3]{0.f, 0.f, 0.f};
		float quat_body_to_ned[4]{1.f, 0.f, 0.f, 0.f};
		float angular_velocity_body[3]{0.f, 0.f, 0.f};

		float target_position_ned[3]{0.f, 0.f, 0.f};
		float target_velocity_ned[3]{0.f, 0.f, 0.f};
		float target_acceleration_ned[3]{0.f, 0.f, 0.f};
		float target_jerk_ned[3]{0.f, 0.f, 0.f};
		float target_snap_ned[3]{0.f, 0.f, 0.f};

		float target_yaw{0.f};
		float target_yaw_rate{0.f};
		float target_yaw_accel{0.f};

		bool state_valid_for_control{false};
		bool armed{false};
		bool failsafe{false};
		uint8_t nav_state{0};
	};

	struct Output {
		hrt_abstime timestamp_us{0};

		float position_error_ned[3]{0.f, 0.f, 0.f};
		float velocity_error_ned[3]{0.f, 0.f, 0.f};
		float desired_acceleration_ned[3]{0.f, 0.f, 0.f};

		float current_euler_rpy[3]{0.f, 0.f, 0.f};
		float desired_euler_rpy[3]{0.f, 0.f, 0.f};
		float attitude_error_rpy[3]{0.f, 0.f, 0.f};
		float attitude_integral_rpy[3]{0.f, 0.f, 0.f};
		float rate_error_body[3]{0.f, 0.f, 0.f};

		float thrust_newton{0.f};
		float moment_newton_meter[3]{0.f, 0.f, 0.f};
		bool valid{false};
	};

	SimplePIDController();
	~SimplePIDController() = default;

	bool update(const Input &input, Output &output);
	void reset();

	const Input &last_input() const { return _last_input; }
	const Output &last_output() const { return _last_output; }

private:
	GeometricController _geometric_controller{};
	Input _last_input{};
	Output _last_output{};
};
