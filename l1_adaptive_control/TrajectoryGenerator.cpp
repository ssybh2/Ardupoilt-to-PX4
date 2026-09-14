#include "TrajectoryGenerator.hpp"

#include "ACRLTrajectories.hpp"
#include "L1SourceConfig.hpp"

#include <matrix/matrix/math.hpp>

using matrix::Vector2f;
using matrix::Vector3f;

namespace
{

void copy3(const Vector3f &input, float output[3])
{
	for (int i = 0; i < 3; i++) {
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

bool TrajectoryGenerator::update(const Input &input, Output &output)
{
	_last_input = input;
	output = Output{};
	output.timestamp_us = input.timestamp_us;

	if (!input.state_valid_for_control || !input.armed || input.failsafe) {
		reset();
		_last_output = output;
		return false;
	}

	float current_time = 0.f;
	float time_in_this_run = 0.f;

	if (!_initialized) {
		_initial_time_us = input.timestamp_us;
		_current_time_last_s = 0.f;
		_time_bias_in_this_run_s = 0.f;
		_initialized = true;

	} else {
		current_time = static_cast<float>(input.timestamp_us - _initial_time_us) * 0.000001f;

		if (current_time - _current_time_last_s <= 0.1f) {
			time_in_this_run = current_time - _time_bias_in_this_run_s;

		} else {
			_time_bias_in_this_run_s = current_time;
			time_in_this_run = 0.f;
		}
	}

	Vector3f target_pos{};
	Vector3f target_vel{};
	Vector3f target_acc{};
	Vector3f target_jerk{};
	Vector3f target_snap{};
	Vector2f target_yaw{1.f, 0.f};
	Vector2f target_yaw_dot{};
	Vector2f target_yaw_ddot{};

	if (time_in_this_run < 2.f) {
		acrl::trajectory_takeoff(time_in_this_run, &target_pos, &target_vel, &target_acc, &target_jerk,
				      &target_snap, &target_yaw, &target_yaw_dot, &target_yaw_ddot);

	} else {
		switch (_parameters.trajectory_index) {
		case 1: {
#if (!REAL_OR_SITL)
			const float time_offset = 2.f;
			acrl::trajectory_circle_variable_yaw(time_in_this_run, _parameters.radius_x, time_offset,
						     _parameters.target_speed, &target_pos, &target_vel, &target_acc,
						     &target_jerk, &target_snap, &target_yaw, &target_yaw_dot,
						     &target_yaw_ddot);
#else
			if (time_in_this_run >= 2.f && time_in_this_run < 4.f) {
				const float time_offset = 2.f;
				acrl::trajectory_transition_to_start(time_in_this_run, _parameters.radius_x, time_offset,
							     &target_pos, &target_vel, &target_acc, &target_jerk,
							     &target_snap, &target_yaw, &target_yaw_dot, &target_yaw_ddot);

			} else if (time_in_this_run >= 4.f) {
				const float time_offset = 4.f;
				acrl::trajectory_circle_variable_yaw(time_in_this_run, _parameters.radius_x, time_offset,
							     _parameters.target_speed, &target_pos, &target_vel,
							     &target_acc, &target_jerk, &target_snap, &target_yaw,
							     &target_yaw_dot, &target_yaw_ddot);
			}
#endif
			break;
		}

		case 2: {
#if (!REAL_OR_SITL)
			const float time_offset = 2.f;
			acrl::trajectory_circle_fixed_yaw(time_in_this_run, _parameters.radius_x, time_offset,
						  _parameters.target_speed, &target_pos, &target_vel, &target_acc,
						  &target_jerk, &target_snap, &target_yaw, &target_yaw_dot,
						  &target_yaw_ddot);
#else
			if (time_in_this_run >= 2.f && time_in_this_run < 4.f) {
				const float time_offset = 2.f;
				acrl::trajectory_transition_to_start(time_in_this_run, _parameters.radius_x, time_offset,
							     &target_pos, &target_vel, &target_acc, &target_jerk,
							     &target_snap, &target_yaw, &target_yaw_dot, &target_yaw_ddot);

			} else if (time_in_this_run >= 4.f) {
				const float time_offset = 4.f;
				acrl::trajectory_circle_fixed_yaw(time_in_this_run, _parameters.radius_x, time_offset,
							  _parameters.target_speed, &target_pos, &target_vel,
							  &target_acc, &target_jerk, &target_snap, &target_yaw,
							  &target_yaw_dot, &target_yaw_ddot);
			}
#endif
			break;
		}

		case 3:
			acrl::trajectory_figure8_fixed_yaw(time_in_this_run, _parameters.radius_x, _parameters.radius_y,
						   _parameters.target_speed, &target_pos, &target_vel, &target_acc,
						   &target_jerk, &target_snap, &target_yaw, &target_yaw_dot,
						   &target_yaw_ddot);
			break;

		case 4:
			acrl::trajectory_figure8_tilted(time_in_this_run, _parameters.radius_x, _parameters.radius_y,
						_parameters.target_speed, &target_pos, &target_vel, &target_acc,
						&target_jerk, &target_snap, &target_yaw, &target_yaw_dot,
						&target_yaw_ddot);
			break;

		default:
			acrl::trajectory_figure8_fixed_yaw(time_in_this_run, _parameters.radius_x, _parameters.radius_y,
						   0.f, &target_pos, &target_vel, &target_acc, &target_jerk,
						   &target_snap, &target_yaw, &target_yaw_dot, &target_yaw_ddot);
			break;
		}
	}

	if (input.land_flag && !_landing_triggered) {
		_landing_triggered = true;
		_landing_time_offset_s = time_in_this_run;
	}

	if (input.land_flag && _landing_triggered) {
		const Vector3f current_position{input.current_position_ned[0], input.current_position_ned[1],
						input.current_position_ned[2]};
		const Vector3f current_velocity{input.current_velocity_ned[0], input.current_velocity_ned[1],
						input.current_velocity_ned[2]};

		if (current_position(2) >= -0.3f) {
			_landing_complete = true;

		} else {
			const float dec_rate = 1.f;
			_landing_complete = acrl::trajectory_land(time_in_this_run - _landing_time_offset_s,
					current_position, current_velocity, input.current_yaw, dec_rate,
					&target_pos, &target_vel, &target_acc, &target_jerk, &target_snap,
					&target_yaw, &target_yaw_dot, &target_yaw_ddot) != 0;
		}
	}

	copy3(target_pos, output.position_ned);
	copy3(target_vel, output.velocity_ned);
	copy3(target_acc, output.acceleration_ned);
	copy3(target_jerk, output.jerk_ned);
	copy3(target_snap, output.snap_ned);
	copy2(target_yaw, output.yaw);
	copy2(target_yaw_dot, output.yaw_dot);
	copy2(target_yaw_ddot, output.yaw_ddot);
	output.current_time_s = current_time;
	output.time_in_this_run_s = time_in_this_run;
	output.landing_triggered = _landing_triggered;
	output.landing_complete = _landing_complete;
	output.valid = true;

	_current_time_last_s = current_time;
	_last_output = output;
	return true;
}

void TrajectoryGenerator::reset()
{
	_initialized = false;
	_initial_time_us = 0;
	_current_time_last_s = 0.f;
	_time_bias_in_this_run_s = 0.f;
	_landing_triggered = false;
	_landing_complete = false;
	_landing_time_offset_s = 0.f;
}
