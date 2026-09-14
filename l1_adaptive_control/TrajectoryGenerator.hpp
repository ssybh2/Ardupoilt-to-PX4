#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

class TrajectoryGenerator
{
public:
	struct Parameters {
		float radius_x{2.0f};       // CIRCRADIUSX_DEFAULT from L1Quad
		float radius_y{1.0f};       // CIRCRADIUSY_DEFAULT from L1Quad
		float target_speed{0.3f};   // CIRCSPEED_DEFAULT from L1Quad
		uint8_t trajectory_index{0}; // TRAJINDEX_DEFAULT from L1Quad
	};

	struct Input {
		hrt_abstime timestamp_us{0};
		float current_position_ned[3]{0.f, 0.f, 0.f};
		float current_velocity_ned[3]{0.f, 0.f, 0.f};
		float current_yaw{0.f};
		bool land_flag{false};
		bool state_valid_for_control{false};
		bool armed{false};
		bool failsafe{false};
	};

	struct Output {
		hrt_abstime timestamp_us{0};
		float position_ned[3]{0.f, 0.f, 0.f};
		float velocity_ned[3]{0.f, 0.f, 0.f};
		float acceleration_ned[3]{0.f, 0.f, 0.f};
		float jerk_ned[3]{0.f, 0.f, 0.f};
		float snap_ned[3]{0.f, 0.f, 0.f};
		float yaw[2]{1.f, 0.f};
		float yaw_dot[2]{0.f, 0.f};
		float yaw_ddot[2]{0.f, 0.f};
		float current_time_s{0.f};
		float time_in_this_run_s{0.f};
		bool landing_triggered{false};
		bool landing_complete{false};
		bool valid{false};
	};

	TrajectoryGenerator() = default;
	~TrajectoryGenerator() = default;

	bool update(const Input &input, Output &output);
	void reset();
	void set_parameters(const Parameters &parameters) { _parameters = parameters; }

	const Parameters &parameters() const { return _parameters; }
	const Input &last_input() const { return _last_input; }
	const Output &last_output() const { return _last_output; }

private:
	Parameters _parameters{};
	Input _last_input{};
	Output _last_output{};

	bool _initialized{false};
	hrt_abstime _initial_time_us{0};
	float _current_time_last_s{0.f};
	float _time_bias_in_this_run_s{0.f};
	bool _landing_triggered{false};
	bool _landing_complete{false};
	float _landing_time_offset_s{0.f};
};
