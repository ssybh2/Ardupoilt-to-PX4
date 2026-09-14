#pragma once

#include "L1SourceConfig.hpp"

#include <matrix/matrix/math.hpp>
#include <stdint.h>

namespace acrl
{

using matrix::Vector2f;
using matrix::Vector3f;

void trajectory_takeoff(float time_in_this_run,
			Vector3f *target_pos,
			Vector3f *target_vel,
			Vector3f *target_acc,
			Vector3f *target_jerk,
			Vector3f *target_snap,
			Vector2f *target_yaw,
			Vector2f *target_yaw_dot,
			Vector2f *target_yaw_ddot);

void trajectory_transition_to_start(float time_in_this_run,
				    float radius_x,
				    float time_offset,
				    Vector3f *target_pos,
				    Vector3f *target_vel,
				    Vector3f *target_acc,
				    Vector3f *target_jerk,
				    Vector3f *target_snap,
				    Vector2f *target_yaw,
				    Vector2f *target_yaw_dot,
				    Vector2f *target_yaw_ddot);

void trajectory_circle_variable_yaw(float time_in_this_run,
				    float radius,
				    float initial_time_offset,
				    float target_speed,
				    Vector3f *target_pos,
				    Vector3f *target_vel,
				    Vector3f *target_acc,
				    Vector3f *target_jerk,
				    Vector3f *target_snap,
				    Vector2f *target_yaw,
				    Vector2f *target_yaw_dot,
				    Vector2f *target_yaw_ddot);

void trajectory_circle_fixed_yaw(float time_in_this_run,
				 float radius,
				 float initial_time_offset,
				 float target_speed,
				 Vector3f *target_pos,
				 Vector3f *target_vel,
				 Vector3f *target_acc,
				 Vector3f *target_jerk,
				 Vector3f *target_snap,
				 Vector2f *target_yaw,
				 Vector2f *target_yaw_dot,
				 Vector2f *target_yaw_ddot);

void trajectory_figure8_fixed_yaw(float time_in_this_run,
				  float radius_x,
				  float radius_y,
				  float target_speed,
				  Vector3f *target_pos,
				  Vector3f *target_vel,
				  Vector3f *target_acc,
				  Vector3f *target_jerk,
				  Vector3f *target_snap,
				  Vector2f *target_yaw,
				  Vector2f *target_yaw_dot,
				  Vector2f *target_yaw_ddot);

void trajectory_figure8_tilted(float time_in_this_run,
			       float radius_x,
			       float radius_y,
			       float target_speed,
			       Vector3f *target_pos,
			       Vector3f *target_vel,
			       Vector3f *target_acc,
			       Vector3f *target_jerk,
			       Vector3f *target_snap,
			       Vector2f *target_yaw,
			       Vector2f *target_yaw_dot,
			       Vector2f *target_yaw_ddot);

uint8_t trajectory_land(float current_time,
			Vector3f current_position,
			Vector3f current_velocity,
			float current_yaw,
			float dec_rate,
			Vector3f *target_pos,
			Vector3f *target_vel,
			Vector3f *target_acc,
			Vector3f *target_jerk,
			Vector3f *target_snap,
			Vector2f *target_yaw,
			Vector2f *target_yaw_dot,
			Vector2f *target_yaw_ddot);

float poly_eval(float poly_coef[], float x, int n);
float poly_diff_eval(float poly_coef[], float x, int n);
float poly_diff2_eval(float poly_coef[], float x, int n);
float poly_diff3_eval(float poly_coef[], float x, int n);
float poly_diff4_eval(float poly_coef[], float x, int n);

} // namespace acrl
