#include "ACRLTrajectories.hpp"

#include <math.h>

namespace acrl
{

void trajectory_takeoff(float time_in_this_run,
			Vector3f *target_pos,
			Vector3f *target_vel,
			Vector3f *target_acc,
			Vector3f *target_jerk,
			Vector3f *target_snap,
			Vector2f *target_yaw,
			Vector2f *target_yaw_dot,
			Vector2f *target_yaw_ddot)
{
	float poly_coef[8] = {-0.1563f, 1.0938f, -2.6250f, 2.1875f, 0.f, 0.f, 0.f, 0.f};

	if (time_in_this_run < 2.f) {
		*target_pos = Vector3f{0.f, 0.f, -poly_eval(poly_coef, time_in_this_run, 8)};
		*target_vel = Vector3f{0.f, 0.f, -poly_diff_eval(poly_coef, time_in_this_run, 8)};
		*target_acc = Vector3f{0.f, 0.f, -poly_diff2_eval(poly_coef, time_in_this_run, 8)};
		*target_jerk = Vector3f{0.f, 0.f, -poly_diff3_eval(poly_coef, time_in_this_run, 8)};
		*target_snap = Vector3f{0.f, 0.f, -poly_diff4_eval(poly_coef, time_in_this_run, 8)};
		*target_yaw = Vector2f{1.f, 0.f};
		*target_yaw_dot = Vector2f{0.f, 0.f};
		*target_yaw_ddot = Vector2f{0.f, 0.f};
	}
}

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
				    Vector2f *target_yaw_ddot)
{
	float poly_coef[8] = {-0.1563f, 1.0938f, -2.6250f, 2.1875f, 0.f, 0.f, 0.f, 0.f};

	if (time_in_this_run >= time_offset && time_in_this_run <= time_offset + 2.f) {
		const float t = time_in_this_run - time_offset;
		*target_pos = Vector3f{0.f, -radius_x * poly_eval(poly_coef, t, 8), -1.f};
		*target_vel = Vector3f{0.f, -radius_x * poly_diff_eval(poly_coef, t, 8), 0.f};
		*target_acc = Vector3f{0.f, -radius_x * poly_diff2_eval(poly_coef, t, 8), 0.f};
		*target_jerk = Vector3f{0.f, -radius_x * poly_diff3_eval(poly_coef, t, 8), 0.f};
		*target_snap = Vector3f{0.f, -radius_x * poly_diff4_eval(poly_coef, t, 8), 0.f};
		*target_yaw = Vector2f{1.f, 0.f};
		*target_yaw_dot = Vector2f{0.f, 0.f};
		*target_yaw_ddot = Vector2f{0.f, 0.f};
	}
}

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
				    Vector2f *target_yaw_ddot)
{
	static float current_speed = 0.f;
	static float time_offset = 0.f;
	static float current_loop_time = 0.f;
	float net_time = 0.f;
	static uint8_t acc_complete = 0;

	if ((current_speed <= 0.1f) && (target_speed >= 0.5f)) {
		current_speed = 0.5f / radius;
		time_offset = initial_time_offset;
		current_loop_time = 6.28f / current_speed;

	} else if ((time_in_this_run - time_offset >= current_loop_time) && (!acc_complete)) {
		if ((current_speed * radius) >= 1.98f) {
			current_speed += 0.1f / radius;

		} else {
			current_speed += 0.5f / radius;
		}

		if (current_speed > (target_speed / radius)) {
			time_offset += current_loop_time;
			current_speed = target_speed / radius;
			acc_complete = 1;

		} else {
			time_offset += current_loop_time;
			current_loop_time = 6.28f / current_speed;
		}
	}

	net_time = time_in_this_run - time_offset;

#if (!REAL_OR_SITL)
	*target_pos = Vector3f{radius * sinf(current_speed * net_time),
			      radius * (1.f - cosf(current_speed * net_time)), -1.f};
#else
	*target_pos = Vector3f{radius * sinf(current_speed * net_time),
			      radius * (-cosf(current_speed * net_time)), -1.f};
#endif

	*target_vel = Vector3f{radius * current_speed * cosf(current_speed * net_time),
			      radius * current_speed * sinf(current_speed * net_time), 0.f};
	*target_acc = Vector3f{-radius * powf(current_speed, 2.f) * sinf(current_speed * net_time),
			      radius * powf(current_speed, 2.f) * cosf(current_speed * net_time), 0.f};
	*target_jerk = Vector3f{-radius * powf(current_speed, 3.f) * cosf(current_speed * net_time),
			       -radius * powf(current_speed, 3.f) * sinf(current_speed * net_time), 0.f};
	*target_snap = Vector3f{radius * powf(current_speed, 4.f) * sinf(current_speed * net_time),
			       -radius * powf(current_speed, 4.f) * cosf(current_speed * net_time), 0.f};
	*target_yaw = Vector2f{cosf(current_speed * net_time), sinf(current_speed * net_time)};
	*target_yaw_dot = Vector2f{-current_speed * sinf(current_speed * net_time),
				  current_speed * cosf(current_speed * net_time)};
	*target_yaw_ddot = Vector2f{-powf(current_speed, 2.f) * cosf(current_speed * net_time),
				   -powf(current_speed, 2.f) * sinf(current_speed * net_time)};
}

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
				 Vector2f *target_yaw_ddot)
{
	static float current_speed = 0.f;
	static float time_offset = 0.f;
	static float current_loop_time = 0.f;
	float net_time = 0.f;
	static uint8_t acc_complete = 0;

	if ((current_speed <= 0.1f) && (target_speed >= 0.5f)) {
		current_speed = 0.5f / radius;
		time_offset = initial_time_offset;
		current_loop_time = 6.28f / current_speed;

	} else if ((time_in_this_run - time_offset >= current_loop_time) && (!acc_complete)) {
		if ((current_speed * radius) >= 1.98f) {
			current_speed += 0.1f / radius;

		} else {
			current_speed += 0.5f / radius;
		}

		if (current_speed > (target_speed / radius)) {
			time_offset += current_loop_time;
			current_speed = target_speed / radius;
			acc_complete = 1;

		} else {
			time_offset += current_loop_time;
			current_loop_time = 6.28f / current_speed;
		}
	}

	net_time = time_in_this_run - time_offset;

#if (!REAL_OR_SITL)
	*target_pos = Vector3f{radius * sinf(current_speed * net_time),
			      radius * (1.f - cosf(current_speed * net_time)), -1.f};
#else
	*target_pos = Vector3f{radius * sinf(current_speed * net_time),
			      radius * (-cosf(current_speed * net_time)), -1.f};
#endif

	*target_vel = Vector3f{radius * current_speed * cosf(current_speed * net_time),
			      radius * current_speed * sinf(current_speed * net_time), 0.f};
	*target_acc = Vector3f{-radius * powf(current_speed, 2.f) * sinf(current_speed * net_time),
			      radius * powf(current_speed, 2.f) * cosf(current_speed * net_time), 0.f};
	*target_jerk = Vector3f{-radius * powf(current_speed, 3.f) * cosf(current_speed * net_time),
			       -radius * powf(current_speed, 3.f) * sinf(current_speed * net_time), 0.f};
	*target_snap = Vector3f{radius * powf(current_speed, 4.f) * sinf(current_speed * net_time),
			       -radius * powf(current_speed, 4.f) * cosf(current_speed * net_time), 0.f};
	*target_yaw = Vector2f{1.f, 0.f};
	*target_yaw_dot = Vector2f{0.f, 0.f};
	*target_yaw_ddot = Vector2f{0.f, 0.f};
}

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
				  Vector2f *target_yaw_ddot)
{
	static float current_rate = 0.f;
	static float current_eqv_rate = 0.f;
	static float time_offset = 0.f;
	static float current_loop_time = 0.f;
	float net_time = 0.f;
	static uint8_t acc_complete = 0;
	static uint8_t positive_loop = 1;
	const float scale_factor = sqrtf(radius_x * radius_x + 4.f * radius_y * radius_y);

	if ((current_rate <= 0.1f) && (target_speed >= 0.5f)) {
		current_rate = 0.5f;
		current_eqv_rate = current_rate / scale_factor;
		time_offset = 2.f;
		current_loop_time = 3.14f / current_eqv_rate;

	} else if ((time_in_this_run - time_offset >= current_loop_time) && (!acc_complete)) {
		if (fabsf(current_rate - target_speed) <= 0.01f) {
			current_rate = target_speed;
			current_eqv_rate = current_rate / scale_factor;
			acc_complete = 1;

		} else {
			current_rate += 0.5f;

			if (current_rate > target_speed) {
				current_rate = target_speed;
			}

			current_eqv_rate = current_rate / scale_factor;
			time_offset += current_loop_time;
			current_loop_time = 3.14f / current_eqv_rate;
			positive_loop = positive_loop ? 0 : 1;
		}
	}

	net_time = time_in_this_run - time_offset;
	const float sf = sinf(current_eqv_rate * net_time);
	const float s2f = sinf(2.f * current_eqv_rate * net_time);
	const float cf = cosf(current_eqv_rate * net_time);
	const float c2f = cosf(2.f * current_eqv_rate * net_time);

	if (positive_loop) {
		*target_pos = Vector3f{radius_x * sf, radius_y * s2f, -1.f};
		*target_vel = Vector3f{radius_x * current_eqv_rate * cf,
				      radius_y * 2.f * current_eqv_rate * c2f, 0.f};
		*target_acc = Vector3f{-radius_x * current_eqv_rate * current_eqv_rate * sf,
				      -radius_y * 4.f * current_eqv_rate * current_eqv_rate * s2f, 0.f};
		*target_jerk = Vector3f{-radius_x * powf(current_eqv_rate, 3.f) * cf,
				       -radius_y * 8.f * powf(current_eqv_rate, 3.f) * c2f, 0.f};
		*target_snap = Vector3f{radius_x * powf(current_eqv_rate, 4.f) * sf,
				       radius_y * 16.f * powf(current_eqv_rate, 4.f) * s2f, 0.f};

	} else {
		*target_pos = Vector3f{-radius_x * sf, radius_y * s2f, -1.f};
		*target_vel = Vector3f{-radius_x * current_eqv_rate * cf,
				      radius_y * 2.f * current_eqv_rate * c2f, 0.f};
		*target_acc = Vector3f{radius_x * current_eqv_rate * current_eqv_rate * sf,
				      -radius_y * 4.f * current_eqv_rate * current_eqv_rate * s2f, 0.f};
		*target_jerk = Vector3f{radius_x * powf(current_eqv_rate, 3.f) * cf,
				       -radius_y * 8.f * powf(current_eqv_rate, 3.f) * c2f, 0.f};
		*target_snap = Vector3f{-radius_x * powf(current_eqv_rate, 4.f) * sf,
				       radius_y * 16.f * powf(current_eqv_rate, 4.f) * s2f, 0.f};
	}

	*target_yaw = Vector2f{1.f, 0.f};
	*target_yaw_dot = Vector2f{0.f, 0.f};
	*target_yaw_ddot = Vector2f{0.f, 0.f};
}

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
			       Vector2f *target_yaw_ddot)
{
	static float current_rate = 0.f;
	static float current_eqv_rate = 0.f;
	static float time_offset = 0.f;
	static float current_loop_time = 0.f;
	float net_time = 0.f;
	static uint8_t acc_complete = 0;
	static uint8_t positive_loop = 1;
	const float z_amp = 0.2f;
	const float scale_factor = sqrtf(radius_x * radius_x + 4.f * radius_y * radius_y + z_amp * z_amp);

	if ((current_rate <= 0.1f) && (target_speed >= 0.5f)) {
		current_rate = 0.5f;
		current_eqv_rate = current_rate / scale_factor;
		time_offset = 2.f;
		current_loop_time = 3.14f / current_eqv_rate;

	} else if ((time_in_this_run - time_offset >= current_loop_time) && (!acc_complete)) {
		if (fabsf(current_rate - target_speed) <= 0.01f) {
			current_rate = target_speed;
			current_eqv_rate = current_rate / scale_factor;
			acc_complete = 1;

		} else {
			current_rate += 0.5f;

			if (current_rate > target_speed) {
				current_rate = target_speed;
			}

			current_eqv_rate = current_rate / scale_factor;
			time_offset += current_loop_time;
			current_loop_time = 3.14f / current_eqv_rate;
			positive_loop = positive_loop ? 0 : 1;
		}
	}

	net_time = time_in_this_run - time_offset;
	const float sf = sinf(current_eqv_rate * net_time);
	const float s2f = sinf(2.f * current_eqv_rate * net_time);
	const float cf = cosf(current_eqv_rate * net_time);
	const float c2f = cosf(2.f * current_eqv_rate * net_time);

	if (positive_loop) {
		*target_pos = Vector3f{radius_x * sf, radius_y * s2f, -1.f - z_amp * sf};
		*target_vel = Vector3f{radius_x * current_eqv_rate * cf,
				      radius_y * 2.f * current_eqv_rate * c2f,
				      -z_amp * current_eqv_rate * cf};
		*target_acc = Vector3f{-radius_x * current_eqv_rate * current_eqv_rate * sf,
				      -radius_y * 4.f * current_eqv_rate * current_eqv_rate * s2f,
				      z_amp * current_eqv_rate * current_eqv_rate * sf};
		*target_jerk = Vector3f{-radius_x * powf(current_eqv_rate, 3.f) * cf,
				       -radius_y * 8.f * powf(current_eqv_rate, 3.f) * c2f,
				       z_amp * powf(current_eqv_rate, 3.f) * cf};
		*target_snap = Vector3f{radius_x * powf(current_eqv_rate, 4.f) * sf,
				       radius_y * 16.f * powf(current_eqv_rate, 4.f) * s2f,
				       -z_amp * powf(current_eqv_rate, 4.f) * sf};

	} else {
		*target_pos = Vector3f{-radius_x * sf, radius_y * s2f, -1.f + z_amp * sf};
		*target_vel = Vector3f{-radius_x * current_eqv_rate * cf,
				      radius_y * 2.f * current_eqv_rate * c2f,
				      z_amp * current_eqv_rate * cf};
		*target_acc = Vector3f{radius_x * current_eqv_rate * current_eqv_rate * sf,
				      -radius_y * 4.f * current_eqv_rate * current_eqv_rate * s2f,
				      -z_amp * current_eqv_rate * current_eqv_rate * sf};
		*target_jerk = Vector3f{radius_x * powf(current_eqv_rate, 3.f) * cf,
				       -radius_y * 8.f * powf(current_eqv_rate, 3.f) * c2f,
				       -z_amp * powf(current_eqv_rate, 3.f) * cf};
		*target_snap = Vector3f{-radius_x * powf(current_eqv_rate, 4.f) * sf,
				       radius_y * 16.f * powf(current_eqv_rate, 4.f) * s2f,
				       z_amp * powf(current_eqv_rate, 4.f) * sf};
	}

	*target_yaw = Vector2f{1.f, 0.f};
	*target_yaw_dot = Vector2f{0.f, 0.f};
	*target_yaw_ddot = Vector2f{0.f, 0.f};
}

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
			Vector2f *target_yaw_ddot)
{
	static Vector3f enter_position{};
	static Vector3f enter_velocity{};
	static float enter_yaw = 0.f;
	static float enter_lin_vel_inv = 0.f;
	static uint8_t dec_complete = 0;
	static float switch_speed = 0.8f;
	static float dec_time = 0.f;
	const float hover_time = 3.f;
	const float land_time = 3.f;
	const float linear_velocity = sqrtf(current_velocity(0) * current_velocity(0)
					   + current_velocity(1) * current_velocity(1)
					   + current_velocity(2) * current_velocity(2));

	if (current_time < 0.0025f) {
		enter_position = current_position;
		enter_velocity = current_velocity;
		enter_yaw = current_yaw;
		enter_lin_vel_inv = 1.f / linear_velocity;
		dec_time = (linear_velocity - switch_speed) / dec_rate;
	}

	if ((linear_velocity > switch_speed) && (!dec_complete)) {
		const float dec_time_inv = 1.f / dec_time;
		*target_vel = Vector3f{
			enter_velocity(0) * (1.f - current_time * dec_time_inv)
			+ switch_speed * enter_velocity(0) * enter_lin_vel_inv * current_time * dec_time_inv,
			enter_velocity(1) * (1.f - current_time * dec_time_inv)
			+ switch_speed * enter_velocity(1) * enter_lin_vel_inv * current_time * dec_time_inv,
			0.f};
		*target_pos = Vector3f{
			enter_position(0) + (enter_velocity(0) + (*target_vel)(0)) * current_time / 2.f,
			enter_position(1) + (enter_velocity(1) + (*target_vel)(1)) * current_time / 2.f,
			enter_position(2)};
		*target_acc = Vector3f{
			-powf(-1.f, signbit(enter_velocity(0))) * dec_time_inv * (1.f - switch_speed * enter_lin_vel_inv),
			-powf(-1.f, signbit(enter_velocity(1))) * dec_time_inv * (1.f - switch_speed * enter_lin_vel_inv),
			0.f};
		*target_jerk = Vector3f{0.f, 0.f, 0.f};
		*target_snap = Vector3f{0.f, 0.f, 0.f};
		*target_yaw = Vector2f{cosf(enter_yaw), sinf(enter_yaw)};
		*target_yaw_dot = Vector2f{0.f, 0.f};
		*target_yaw_ddot = Vector2f{0.f, 0.f};
		return 0;
	}

	if (!dec_complete) {
		dec_complete = 1;
		enter_position = current_position;
		enter_velocity = current_velocity;
		enter_yaw = current_yaw;
		dec_time = current_time;
	}

	if (current_time <= dec_time + hover_time) {
		*target_pos = enter_position;
		*target_vel = Vector3f{0.f, 0.f, 0.f};
		*target_acc = Vector3f{0.f, 0.f, 0.f};
		*target_jerk = Vector3f{0.f, 0.f, 0.f};
		*target_snap = Vector3f{0.f, 0.f, 0.f};
		*target_yaw = Vector2f{cosf(enter_yaw), sinf(enter_yaw)};
		*target_yaw_dot = Vector2f{0.f, 0.f};
		*target_yaw_ddot = Vector2f{0.f, 0.f};
		return 0;
	}

	if (current_time <= dec_time + hover_time + land_time) {
		float poly_coef[8] = {-0.00122109375f, 0.017090625f, -0.08203125f, 0.13671875f,
				      0.f, 0.f, 0.f, 0.f};
		const float t = current_time - dec_time - hover_time;
		*target_pos = Vector3f{enter_position(0), enter_position(1),
				      enter_position(2) - enter_position(2) * poly_eval(poly_coef, t, 8)};
		*target_vel = Vector3f{0.f, 0.f, -enter_position(2) * poly_diff_eval(poly_coef, t, 8)};
		*target_acc = Vector3f{0.f, 0.f, -enter_position(2) * poly_diff2_eval(poly_coef, t, 8)};
		*target_jerk = Vector3f{0.f, 0.f, -enter_position(2) * poly_diff3_eval(poly_coef, t, 8)};
		*target_snap = Vector3f{0.f, 0.f, -enter_position(2) * poly_diff4_eval(poly_coef, t, 8)};
		*target_yaw = Vector2f{cosf(enter_yaw), sinf(enter_yaw)};
		*target_yaw_dot = Vector2f{0.f, 0.f};
		*target_yaw_ddot = Vector2f{0.f, 0.f};
		return 0;
	}

	*target_pos = Vector3f{enter_position(0), enter_position(1), 0.f};
	*target_vel = Vector3f{0.f, 0.f, 0.f};
	*target_acc = Vector3f{0.f, 0.f, 0.f};
	*target_jerk = Vector3f{0.f, 0.f, 0.f};
	*target_snap = Vector3f{0.f, 0.f, 0.f};
	*target_yaw = Vector2f{cosf(enter_yaw), sinf(enter_yaw)};
	*target_yaw_dot = Vector2f{0.f, 0.f};
	*target_yaw_ddot = Vector2f{0.f, 0.f};
	return 1;
}

float poly_eval(float poly_coef[], float x, int n)
{
	float result = 0.f;

	for (int i = 0; i < n; i++) {
		result += poly_coef[i] * powf(x, n - 1 - i);
	}

	return result;
}

float poly_diff_eval(float poly_coef[], float x, int n)
{
	float result = 0.f;

	for (int i = 0; i < n - 1; i++) {
		result += poly_coef[i] * powf(x, n - 2 - i) * static_cast<float>(n - 1 - i);
	}

	return result;
}

float poly_diff2_eval(float poly_coef[], float x, int n)
{
	float result = 0.f;

	for (int i = 0; i < n - 2; i++) {
		result += poly_coef[i] * powf(x, n - 3 - i)
			  * static_cast<float>(n - 1 - i) * static_cast<float>(n - 2 - i);
	}

	return result;
}

float poly_diff3_eval(float poly_coef[], float x, int n)
{
	float result = 0.f;

	for (int i = 0; i < n - 3; i++) {
		result += poly_coef[i] * powf(x, n - 4 - i)
			  * static_cast<float>(n - 1 - i) * static_cast<float>(n - 2 - i)
			  * static_cast<float>(n - 3 - i);
	}

	return result;
}

float poly_diff4_eval(float poly_coef[], float x, int n)
{
	float result = 0.f;

	for (int i = 0; i < n - 4; i++) {
		result += poly_coef[i] * powf(x, n - 5 - i)
			  * static_cast<float>(n - 1 - i) * static_cast<float>(n - 2 - i)
			  * static_cast<float>(n - 3 - i) * static_cast<float>(n - 4 - i);
	}

	return result;
}

} // namespace acrl
