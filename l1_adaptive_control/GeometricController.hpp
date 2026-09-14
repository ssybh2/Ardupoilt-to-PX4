#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

class GeometricController
{
public:
	// Defaults are the real-aircraft values from the uploaded DSun controller and
	// the REAL_OR_SITL branch of L1Quad.  The PX4 orchestrator may explicitly
	// load the SITL source values when REAL_OR_SITL == 0.
	struct Parameters {
		float mass_kg{0.62f};
		float position_gain[3]{14.0f, 15.0f, 15.0f};
		float velocity_gain[3]{1.5f, 0.9f, 1.1f};
		float rotation_gain[3]{0.55f, 0.35f, 0.15f};
		float angular_velocity_gain[3]{0.035f, 0.03f, 0.004f};
		float inertia_kg_m2[3]{0.002016f, 0.001827f, 0.00322f};
	};

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

		// Keep the original ModeAdaptive API: yaw is represented by
		// [cos(yaw), sin(yaw)] and its first/second derivatives.
		float target_yaw[2]{1.f, 0.f};
		float target_yaw_dot[2]{0.f, 0.f};
		float target_yaw_ddot[2]{0.f, 0.f};

		bool state_valid_for_control{false};
		bool armed{false};
		bool failsafe{false};
		uint8_t nav_state{0};
	};

	struct Output {
		hrt_abstime timestamp_us{0};
		float r_error[3]{0.f, 0.f, 0.f};
		float v_error[3]{0.f, 0.f, 0.f};
		float target_thrust{0.f};
		float M[3]{0.f, 0.f, 0.f};
		bool valid{false};
	};

	GeometricController() = default;
	~GeometricController() = default;

	bool update(const Input &input, Output &output);
	void set_parameters(const Parameters &parameters) { _parameters = parameters; }

	const Input &last_input() const { return _last_input; }
	const Output &last_output() const { return _last_output; }
	const Parameters &parameters() const { return _parameters; }

private:
	Parameters _parameters{};
	Input _last_input{};
	Output _last_output{};
};
