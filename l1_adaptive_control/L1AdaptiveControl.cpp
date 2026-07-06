#include "L1AdaptiveControl.hpp"

#include <math.h>
#include <mathlib/mathlib.h>
#include <string.h>

namespace
{

static constexpr float VEHICLE_MASS_KG = 3.0f;
static constexpr float GRAVITY_MSS = 9.80665f;

static constexpr float JXX_KGM2 = 0.023f;
static constexpr float JYY_KGM2 = 0.023f;
static constexpr float JZZ_KGM2 = 0.0459f;

static constexpr float JINV_XX = 43.478f;
static constexpr float JINV_YY = 43.478f;
static constexpr float JINV_ZZ = 21.786f;

static constexpr bool L1_ENABLE = true;
static constexpr float L1_AS_V = -5.0f;
static constexpr float L1_AS_OMEGA = -10.0f;
static constexpr float L1_CUTOFF_Q1_THRUST = 10.0f;
static constexpr float L1_CUTOFF_Q1_MOMENT = 10.0f;
static constexpr float L1_CUTOFF_Q2_MOMENT = 2.0f;

static constexpr float MAX_THRUST_N = VEHICLE_MASS_KG * GRAVITY_MSS * 2.0f;
static constexpr float MAX_ROLL_PITCH_MOMENT_NM = 2.0f;
static constexpr float MAX_YAW_MOMENT_NM = 1.0f;

static constexpr float MAX_L1_THRUST_N = VEHICLE_MASS_KG * GRAVITY_MSS * 0.35f;
static constexpr float MAX_L1_ROLL_PITCH_MOMENT_NM = 0.35f;
static constexpr float MAX_L1_YAW_MOMENT_NM = 0.20f;
static constexpr hrt_abstime MANUAL_CONTROL_TIMEOUT_US = 500000;

float dot3(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void cross3(const float a[3], const float b[3], float out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

void quat_to_rotation_matrix_body_to_ned(const float q[4], float R[3][3])
{
	const float w = q[0];
	const float x = q[1];
	const float y = q[2];
	const float z = q[3];

	R[0][0] = 1.0f - 2.0f * (y * y + z * z);
	R[0][1] = 2.0f * (x * y - w * z);
	R[0][2] = 2.0f * (x * z + w * y);

	R[1][0] = 2.0f * (x * y + w * z);
	R[1][1] = 1.0f - 2.0f * (x * x + z * z);
	R[1][2] = 2.0f * (y * z - w * x);

	R[2][0] = 2.0f * (x * z - w * y);
	R[2][1] = 2.0f * (y * z + w * x);
	R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

float yaw_from_quat_body_to_ned(const float q[4])
{
	float R[3][3]{};
	quat_to_rotation_matrix_body_to_ned(q, R);
	return atan2f(R[1][0], R[0][0]);
}

void get_matrix_column(const float R[3][3], int col, float out[3])
{
	out[0] = R[0][col];
	out[1] = R[1][col];
	out[2] = R[2][col];
}

void copy3(const float in[3], float out[3])
{
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}

float phi_inverse_mu(float prediction_error, float as_value, float dt)
{
	const float exp_as_dt = expf(as_value * dt);
	const float denominator = exp_as_dt - 1.0f;

	if (fabsf(denominator) < 1e-5f || fabsf(as_value) < 1e-5f) {
		return 0.f;
	}

	return prediction_error / denominator * as_value * exp_as_dt;
}

}

L1AdaptiveControl::L1AdaptiveControl() :
ModuleParams(nullptr),
ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

L1AdaptiveControl::~L1AdaptiveControl()
{
perf_free(_loop_perf);
perf_free(_loop_interval_perf);
}

bool L1AdaptiveControl::init()
{
PX4_INFO("L1 adaptive control init");

// 250 Hz control loop. This matches the PX4 control allocation path better than
// the earlier debug-only 10 Hz loop while keeping the module independent.
ScheduleOnInterval(4_ms);

return true;
}

void L1AdaptiveControl::Run()
{
if (should_exit()) {
ScheduleClear();
exit_and_cleanup();
return;
}

perf_begin(_loop_perf);
perf_count(_loop_interval_perf);

update_subscriptions();
update_internal_state();

update_trajectory_input();
run_trajectory_generator();

update_controller_input();
run_geometric_controller();
run_l1_adaptive_augmentation();
publish_control_setpoints();

const hrt_abstime now_us = hrt_absolute_time();

if (now_us - _last_print_us > 1000000) {
print_debug_info();
_last_print_us = now_us;
}

perf_end(_loop_perf);
}

void L1AdaptiveControl::update_subscriptions()
{
if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
_has_local_position = true;
}

if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
_has_attitude = true;
}

if (_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)) {
_has_angular_velocity = true;
}

if (_manual_control_setpoint_sub.update(&_manual_control_setpoint)) {
_has_manual_control_setpoint = true;
}

if (_vehicle_status_sub.update(&_vehicle_status)) {
_has_vehicle_status = true;
}
}

void L1AdaptiveControl::update_internal_state()
{
_state.timestamp_us = hrt_absolute_time();

if (_has_vehicle_status) {
_state.arming_state = _vehicle_status.arming_state;
_state.nav_state = _vehicle_status.nav_state;
_state.failsafe = _vehicle_status.failsafe;
_state.armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
}

if (_has_local_position) {
_state.position_ned[0] = _vehicle_local_position.x;
_state.position_ned[1] = _vehicle_local_position.y;
_state.position_ned[2] = _vehicle_local_position.z;

_state.velocity_ned[0] = _vehicle_local_position.vx;
_state.velocity_ned[1] = _vehicle_local_position.vy;
_state.velocity_ned[2] = _vehicle_local_position.vz;

_state.position_valid = _vehicle_local_position.xy_valid && _vehicle_local_position.z_valid;
_state.velocity_valid = _vehicle_local_position.v_xy_valid && _vehicle_local_position.v_z_valid;
}

if (_has_attitude) {
_state.quat_body_to_ned[0] = _vehicle_attitude.q[0];
_state.quat_body_to_ned[1] = _vehicle_attitude.q[1];
_state.quat_body_to_ned[2] = _vehicle_attitude.q[2];
_state.quat_body_to_ned[3] = _vehicle_attitude.q[3];

_state.attitude_valid = true;
}

if (_has_angular_velocity) {
_state.angular_velocity_body[0] = _vehicle_angular_velocity.xyz[0];
_state.angular_velocity_body[1] = _vehicle_angular_velocity.xyz[1];
_state.angular_velocity_body[2] = _vehicle_angular_velocity.xyz[2];

_state.angular_velocity_valid = true;
}

_state_valid_for_control =
_state.position_valid &&
_state.velocity_valid &&
_state.attitude_valid &&
_state.angular_velocity_valid &&
!_state.failsafe;
}

void L1AdaptiveControl::update_trajectory_input()
{
_trajectory_input.timestamp_us = _state.timestamp_us;

for (int i = 0; i < 3; i++) {
_trajectory_input.current_position_ned[i] = _state.position_ned[i];
}

_trajectory_input.current_yaw = yaw_from_quat_body_to_ned(_state.quat_body_to_ned);

_trajectory_input.state_valid_for_control = _state_valid_for_control;
_trajectory_input.armed = _state.armed;
_trajectory_input.failsafe = _state.failsafe;
_trajectory_input.nav_state = _state.nav_state;

update_manual_height_control_input();
_trajectory_input.manual_height_control_enabled = _rc_height_control_enabled.load();
_trajectory_input.manual_height_control_valid = _manual_height_control_valid;
_trajectory_input.manual_height_stick = _manual_height_stick;
}

void L1AdaptiveControl::run_trajectory_generator()
{
_trajectory_update_executed = _trajectory_generator.update(_trajectory_input, _trajectory_output);
}

void L1AdaptiveControl::update_manual_height_control_input()
{
_manual_height_control_valid = false;
_manual_height_stick = 0.f;

if (!_rc_height_control_enabled.load() || !_has_manual_control_setpoint || !_manual_control_setpoint.valid) {
return;
}

if (_manual_control_setpoint.timestamp == 0 || _manual_control_setpoint.timestamp > _state.timestamp_us) {
return;
}

if (_state.timestamp_us - _manual_control_setpoint.timestamp > MANUAL_CONTROL_TIMEOUT_US) {
return;
}

if (!PX4_ISFINITE(_manual_control_setpoint.throttle)) {
return;
}

_manual_height_stick = math::constrain(_manual_control_setpoint.throttle, -1.f, 1.f);
_manual_height_control_valid = true;
}

void L1AdaptiveControl::update_controller_input()
{
_controller_input.timestamp_us = _state.timestamp_us;

for (int i = 0; i < 3; i++) {
_controller_input.position_ned[i] = _state.position_ned[i];
_controller_input.velocity_ned[i] = _state.velocity_ned[i];
_controller_input.angular_velocity_body[i] = _state.angular_velocity_body[i];

_controller_input.target_position_ned[i] = _trajectory_output.position_ned[i];
_controller_input.target_velocity_ned[i] = _trajectory_output.velocity_ned[i];
_controller_input.target_acceleration_ned[i] = _trajectory_output.acceleration_ned[i];
_controller_input.target_jerk_ned[i] = _trajectory_output.jerk_ned[i];
_controller_input.target_snap_ned[i] = _trajectory_output.snap_ned[i];
}

for (int i = 0; i < 4; i++) {
_controller_input.quat_body_to_ned[i] = _state.quat_body_to_ned[i];
}

_controller_input.target_yaw = _trajectory_output.yaw;
_controller_input.target_yaw_rate = _trajectory_output.yaw_rate;
_controller_input.target_yaw_accel = _trajectory_output.yaw_accel;

_controller_input.state_valid_for_control = _state_valid_for_control && _trajectory_output.valid;
_controller_input.armed = _state.armed;
_controller_input.failsafe = _state.failsafe;
_controller_input.nav_state = _state.nav_state;
}

void L1AdaptiveControl::run_geometric_controller()
{
_geometric_update_executed = _geometric_controller.update(_controller_input, _geometric_output);
}

void L1AdaptiveControl::reset_l1_adaptive_state()
{
	_l1_state = L1AdaptiveState{};

	for (int i = 0; i < 4; i++) {
		_l1_output_thrust_moment[i] = 0.f;
	}
}

void L1AdaptiveControl::run_l1_adaptive_augmentation()
{
	_l1_update_executed = false;

	for (int i = 0; i < 4; i++) {
		_l1_output_thrust_moment[i] = 0.f;
		_combined_thrust_moment[i] = 0.f;
	}

	if (!_geometric_output.valid || !_state_valid_for_control || !_state.armed || _state.failsafe) {
		reset_l1_adaptive_state();
		return;
	}

	float R[3][3]{};
	quat_to_rotation_matrix_body_to_ned(_state.quat_body_to_ned, R);

	const float baseline_thrust_moment[4] = {
		_geometric_output.thrust_newton,
		_geometric_output.moment_newton_meter[0],
		_geometric_output.moment_newton_meter[1],
		_geometric_output.moment_newton_meter[2]
	};

	if (!_l1_state.initialized) {
		copy3(_state.velocity_ned, _l1_state.velocity_hat_prev);
		copy3(_state.velocity_ned, _l1_state.velocity_prev);
		copy3(_state.angular_velocity_body, _l1_state.angular_velocity_hat_prev);
		copy3(_state.angular_velocity_body, _l1_state.angular_velocity_prev);

		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				_l1_state.rotation_body_to_ned_prev[row][col] = R[row][col];
			}
		}

		for (int i = 0; i < 4; i++) {
			_l1_state.baseline_thrust_moment_prev[i] = baseline_thrust_moment[i];
		}

		_l1_state.last_update_us = _state.timestamp_us;
		_l1_state.initialized = true;
	}

	const float dt = math::constrain((_state.timestamp_us - _l1_state.last_update_us) * 1e-6f, 0.001f, 0.02f);

	float colx_prev[3]{};
	float coly_prev[3]{};
	float colz_prev[3]{};
	get_matrix_column(_l1_state.rotation_body_to_ned_prev, 0, colx_prev);
	get_matrix_column(_l1_state.rotation_body_to_ned_prev, 1, coly_prev);
	get_matrix_column(_l1_state.rotation_body_to_ned_prev, 2, colz_prev);

	float velocity_prediction_error_prev[3]{};
	float angular_prediction_error_prev[3]{};

	for (int i = 0; i < 3; i++) {
		velocity_prediction_error_prev[i] = _l1_state.velocity_hat_prev[i] - _l1_state.velocity_prev[i];
		angular_prediction_error_prev[i] = _l1_state.angular_velocity_hat_prev[i] - _l1_state.angular_velocity_prev[i];
	}

	const float previous_total_thrust =
		_l1_state.baseline_thrust_moment_prev[0]
		+ _l1_state.adaptive_thrust_moment_prev[0]
		+ _l1_state.sigma_matched_prev[0];

	float velocity_hat[3]{};

	for (int i = 0; i < 3; i++) {
		const float gravity_term = (i == 2) ? GRAVITY_MSS : 0.f;

		velocity_hat[i] = _l1_state.velocity_hat_prev[i]
				  + (gravity_term
				     - colz_prev[i] * previous_total_thrust / VEHICLE_MASS_KG
				     + colx_prev[i] * _l1_state.sigma_unmatched_prev[0] / VEHICLE_MASS_KG
				     + coly_prev[i] * _l1_state.sigma_unmatched_prev[1] / VEHICLE_MASS_KG
				     + velocity_prediction_error_prev[i] * L1_AS_V) * dt;
	}

	const float omega_prev[3] = {
		_l1_state.angular_velocity_prev[0],
		_l1_state.angular_velocity_prev[1],
		_l1_state.angular_velocity_prev[2]
	};

	const float j_omega_prev[3] = {
		JXX_KGM2 * omega_prev[0],
		JYY_KGM2 * omega_prev[1],
		JZZ_KGM2 * omega_prev[2]
	};

	float gyro_moment_prev[3]{};
	cross3(omega_prev, j_omega_prev, gyro_moment_prev);

	const float previous_total_moment[3] = {
		_l1_state.baseline_thrust_moment_prev[1] + _l1_state.adaptive_thrust_moment_prev[1] + _l1_state.sigma_matched_prev[1],
		_l1_state.baseline_thrust_moment_prev[2] + _l1_state.adaptive_thrust_moment_prev[2] + _l1_state.sigma_matched_prev[2],
		_l1_state.baseline_thrust_moment_prev[3] + _l1_state.adaptive_thrust_moment_prev[3] + _l1_state.sigma_matched_prev[3]
	};

	const float inertia_inverse[3] = {JINV_XX, JINV_YY, JINV_ZZ};

	float angular_velocity_hat[3]{};

	for (int i = 0; i < 3; i++) {
		angular_velocity_hat[i] = _l1_state.angular_velocity_hat_prev[i]
					  + (-inertia_inverse[i] * gyro_moment_prev[i]
					     + inertia_inverse[i] * previous_total_moment[i]
					     + angular_prediction_error_prev[i] * L1_AS_OMEGA) * dt;
	}

	float velocity_prediction_error[3]{};
	float angular_prediction_error[3]{};
	float phi_inv_mu_v[3]{};
	float phi_inv_mu_omega[3]{};

	for (int i = 0; i < 3; i++) {
		velocity_prediction_error[i] = velocity_hat[i] - _state.velocity_ned[i];
		angular_prediction_error[i] = angular_velocity_hat[i] - _state.angular_velocity_body[i];
		phi_inv_mu_v[i] = phi_inverse_mu(velocity_prediction_error[i], L1_AS_V, dt);
		phi_inv_mu_omega[i] = phi_inverse_mu(angular_prediction_error[i], L1_AS_OMEGA, dt);
	}

	float colx[3]{};
	float coly[3]{};
	float colz[3]{};
	get_matrix_column(R, 0, colx);
	get_matrix_column(R, 1, coly);
	get_matrix_column(R, 2, colz);

	float sigma_matched[4]{};
	float sigma_unmatched[2]{};

	sigma_matched[0] = dot3(colz, phi_inv_mu_v) * VEHICLE_MASS_KG;
	sigma_matched[1] = -JXX_KGM2 * phi_inv_mu_omega[0];
	sigma_matched[2] = -JYY_KGM2 * phi_inv_mu_omega[1];
	sigma_matched[3] = -JZZ_KGM2 * phi_inv_mu_omega[2];

	sigma_unmatched[0] = -dot3(colx, phi_inv_mu_v) * VEHICLE_MASS_KG;
	sigma_unmatched[1] = -dot3(coly, phi_inv_mu_v) * VEHICLE_MASS_KG;

	const float lpf1_thrust_keep = expf(-L1_CUTOFF_Q1_THRUST * dt);
	const float lpf1_moment_keep = expf(-L1_CUTOFF_Q1_MOMENT * dt);
	const float lpf2_moment_keep = expf(-L1_CUTOFF_Q2_MOMENT * dt);

	float lpf1[4]{};
	float lpf2[4]{};

	lpf1[0] = lpf1_thrust_keep * _l1_state.lpf1_prev[0] + (1.f - lpf1_thrust_keep) * sigma_matched[0];

	for (int i = 1; i < 4; i++) {
		lpf1[i] = lpf1_moment_keep * _l1_state.lpf1_prev[i] + (1.f - lpf1_moment_keep) * sigma_matched[i];
		lpf2[i] = lpf2_moment_keep * _l1_state.lpf2_prev[i] + (1.f - lpf2_moment_keep) * lpf1[i];
	}

	lpf2[0] = lpf1[0];

	_l1_output_thrust_moment[0] = L1_ENABLE ? math::constrain(-lpf2[0], -MAX_L1_THRUST_N, MAX_L1_THRUST_N) : 0.f;
	_l1_output_thrust_moment[1] = L1_ENABLE ? math::constrain(-lpf2[1], -MAX_L1_ROLL_PITCH_MOMENT_NM, MAX_L1_ROLL_PITCH_MOMENT_NM) : 0.f;
	_l1_output_thrust_moment[2] = L1_ENABLE ? math::constrain(-lpf2[2], -MAX_L1_ROLL_PITCH_MOMENT_NM, MAX_L1_ROLL_PITCH_MOMENT_NM) : 0.f;
	_l1_output_thrust_moment[3] = L1_ENABLE ? math::constrain(-lpf2[3], -MAX_L1_YAW_MOMENT_NM, MAX_L1_YAW_MOMENT_NM) : 0.f;

	for (int i = 0; i < 4; i++) {
		_l1_state.baseline_thrust_moment_prev[i] = baseline_thrust_moment[i];
		_l1_state.adaptive_thrust_moment_prev[i] = _l1_output_thrust_moment[i];
		_l1_state.sigma_matched_prev[i] = sigma_matched[i];
		_l1_state.lpf1_prev[i] = lpf1[i];
		_l1_state.lpf2_prev[i] = lpf2[i];
		_combined_thrust_moment[i] = baseline_thrust_moment[i] + _l1_output_thrust_moment[i];
	}

	sigma_unmatched[0] = math::constrain(sigma_unmatched[0], -MAX_L1_THRUST_N, MAX_L1_THRUST_N);
	sigma_unmatched[1] = math::constrain(sigma_unmatched[1], -MAX_L1_THRUST_N, MAX_L1_THRUST_N);

	_l1_state.sigma_unmatched_prev[0] = sigma_unmatched[0];
	_l1_state.sigma_unmatched_prev[1] = sigma_unmatched[1];

	copy3(velocity_hat, _l1_state.velocity_hat_prev);
	copy3(_state.velocity_ned, _l1_state.velocity_prev);
	copy3(angular_velocity_hat, _l1_state.angular_velocity_hat_prev);
	copy3(_state.angular_velocity_body, _l1_state.angular_velocity_prev);

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			_l1_state.rotation_body_to_ned_prev[row][col] = R[row][col];
		}
	}

	_l1_state.last_update_us = _state.timestamp_us;
	_l1_update_executed = true;
}

void L1AdaptiveControl::publish_control_setpoints()
{
	_control_setpoint_published = false;

	if (!_geometric_output.valid || !_state_valid_for_control || !_state.armed || _state.failsafe) {
		for (int i = 0; i < 3; i++) {
			_published_thrust_body[i] = 0.f;
			_published_torque_body[i] = 0.f;
		}

		return;
	}

	const float thrust_n = math::constrain(_combined_thrust_moment[0], 0.f, MAX_THRUST_N);

	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.timestamp = hrt_absolute_time();
	thrust_sp.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	thrust_sp.xyz[0] = 0.f;
	thrust_sp.xyz[1] = 0.f;
	thrust_sp.xyz[2] = -math::constrain(thrust_n / MAX_THRUST_N, 0.f, 1.f);

	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp = thrust_sp.timestamp;
	torque_sp.timestamp_sample = thrust_sp.timestamp_sample;
	torque_sp.xyz[0] = math::constrain(_combined_thrust_moment[1] / MAX_ROLL_PITCH_MOMENT_NM, -1.f, 1.f);
	torque_sp.xyz[1] = math::constrain(_combined_thrust_moment[2] / MAX_ROLL_PITCH_MOMENT_NM, -1.f, 1.f);
	torque_sp.xyz[2] = math::constrain(_combined_thrust_moment[3] / MAX_YAW_MOMENT_NM, -1.f, 1.f);

	_vehicle_thrust_setpoint_pub.publish(thrust_sp);
	_vehicle_torque_setpoint_pub.publish(torque_sp);

	for (int i = 0; i < 3; i++) {
		_published_thrust_body[i] = thrust_sp.xyz[i];
		_published_torque_body[i] = torque_sp.xyz[i];
	}

	_control_setpoint_published = true;
	_control_setpoint_publish_count++;
}

void L1AdaptiveControl::print_debug_info()
{
PX4_INFO("L1 | armed=%d failsafe=%d nav=%u valid=%d | traj=%u valid=%d t=%.1fs",
 (int)_state.armed,
 (int)_state.failsafe,
 (unsigned)_state.nav_state,
 (int)_state_valid_for_control,
 (unsigned)_trajectory_output.mode,
 (int)_trajectory_output.valid,
 (double)_trajectory_output.elapsed_time_s);

PX4_INFO("   pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] target_z=%.2f target_vz=%.2f",
 (double)_state.position_ned[0],
 (double)_state.position_ned[1],
 (double)_state.position_ned[2],
 (double)_state.velocity_ned[0],
 (double)_state.velocity_ned[1],
 (double)_state.velocity_ned[2],
 (double)_trajectory_output.position_ned[2],
 (double)_trajectory_output.velocity_ned[2]);

PX4_INFO("   rc_height=%d manual=%d valid=%d throttle=%.2f | subs lp/att/omega/status=%d/%d/%d/%d",
 (int)_rc_height_control_enabled.load(),
 (int)_has_manual_control_setpoint,
 (int)_manual_height_control_valid,
 (double)_manual_height_stick,
 (int)_has_local_position,
 (int)_has_attitude,
 (int)_has_angular_velocity,
 (int)_has_vehicle_status);

PX4_INFO("   controller: traj=%d geo=%d l1=%d publish=%d count=%u",
 (int)_trajectory_update_executed,
 (int)_geometric_update_executed,
 (int)_l1_update_executed,
 (int)_control_setpoint_published,
 (unsigned)_control_setpoint_publish_count);

PX4_INFO("   output: F=%.2fN M=[%.2f %.2f %.2f] thrust_z=%.3f torque=[%.3f %.3f %.3f]",
 (double)_combined_thrust_moment[0],
 (double)_combined_thrust_moment[1],
 (double)_combined_thrust_moment[2],
 (double)_combined_thrust_moment[3],
 (double)_published_thrust_body[2],
 (double)_published_torque_body[0],
 (double)_published_torque_body[1],
 (double)_published_torque_body[2]);
}

int L1AdaptiveControl::task_spawn(int argc, char *argv[])
{
L1AdaptiveControl *instance = new L1AdaptiveControl();

if (instance) {
_object.store(instance);
_task_id = task_id_is_work_queue;

if (instance->init()) {
return PX4_OK;
}

} else {
PX4_ERR("alloc failed");
}

delete instance;
_object.store(nullptr);
_task_id = -1;

return PX4_ERROR;
}

int L1AdaptiveControl::print_status()
{
PX4_INFO("L1 adaptive control");
PX4_INFO("  state: armed=%d failsafe=%d nav=%u valid=%d",
 (int)_state.armed,
 (int)_state.failsafe,
 (unsigned)_state.nav_state,
 (int)_state_valid_for_control);

PX4_INFO("  trajectory: mode=%u valid=%d target_z=%.3f target_vz=%.3f elapsed=%.2fs",
 (unsigned)_trajectory_output.mode,
 (int)_trajectory_output.valid,
 (double)_trajectory_output.position_ned[2],
 (double)_trajectory_output.velocity_ned[2],
 (double)_trajectory_output.elapsed_time_s);

PX4_INFO("  rc height: enabled=%d received=%d valid=%d throttle=%.3f",
 (int)_rc_height_control_enabled.load(),
 (int)_has_manual_control_setpoint,
 (int)_manual_height_control_valid,
 (double)_manual_height_stick);

PX4_INFO("  pipeline: traj=%d geo=%d l1=%d publish=%d count=%u",
 (int)_trajectory_update_executed,
 (int)_geometric_update_executed,
 (int)_l1_update_executed,
 (int)_control_setpoint_published,
 (unsigned)_control_setpoint_publish_count);

PX4_INFO("  subscriptions: local_pos=%d attitude=%d angular_vel=%d manual=%d status=%d",
 (int)_has_local_position,
 (int)_has_attitude,
 (int)_has_angular_velocity,
 (int)_has_manual_control_setpoint,
 (int)_has_vehicle_status);

perf_print_counter(_loop_perf);
perf_print_counter(_loop_interval_perf);

return 0;
}

void L1AdaptiveControl::set_rc_height_control_enabled(bool enabled)
{
_rc_height_control_enabled.store(enabled);
_manual_height_control_valid = false;
_manual_height_stick = 0.f;
}

int L1AdaptiveControl::custom_command(int argc, char *argv[])
{
if (argc >= 1 && !strcmp(argv[0], "rc_control")) {
if (!is_running()) {
PX4_ERR("module not running");
return -1;
}

L1AdaptiveControl *instance = get_instance();

if (instance == nullptr) {
PX4_ERR("module instance unavailable");
return -1;
}

if (argc < 2 || !strcmp(argv[1], "status")) {
PX4_INFO("RC height control: %s", instance->rc_height_control_enabled() ? "enabled" : "disabled");
PX4_INFO("  manual: received=%d valid=%d throttle=%.3f",
 (int)instance->_has_manual_control_setpoint,
 (int)instance->_manual_height_control_valid,
 (double)instance->_manual_height_stick);
PX4_INFO("  trajectory: mode=%u target_z=%.3f target_vz=%.3f",
 (unsigned)instance->_trajectory_output.mode,
 (double)instance->_trajectory_output.position_ned[2],
 (double)instance->_trajectory_output.velocity_ned[2]);
return 0;
}

if (!strcmp(argv[1], "enable") || !strcmp(argv[1], "1") || !strcmp(argv[1], "true")) {
instance->set_rc_height_control_enabled(true);
PX4_INFO("RC height control enabled");
return 0;
}

if (!strcmp(argv[1], "disable") || !strcmp(argv[1], "0") || !strcmp(argv[1], "false")) {
instance->set_rc_height_control_enabled(false);
PX4_INFO("RC height control disabled");
return 0;
}

return print_usage("unknown rc_control argument");
}

return print_usage("unknown command");
}

int L1AdaptiveControl::print_usage(const char *reason)
{
if (reason) {
PX4_WARN("%s\n", reason);
}

PRINT_MODULE_DESCRIPTION(
R"DESCR_STR(
### Description
L1 adaptive control module skeleton for PX4 v1.17.0.

Current stage:
- Subscribe vehicle_local_position
- Subscribe vehicle_attitude
- Subscribe vehicle_angular_velocity
- Subscribe manual_control_setpoint
- Subscribe vehicle_status
- Convert uORB messages into internal controller state
- Generate takeoff-and-hold trajectory
- Convert trajectory output into GeometricController input
- Run geometric controller
- Run L1 adaptive augmentation
- Publish vehicle_thrust_setpoint and vehicle_torque_setpoint for PX4 control_allocator
)DESCR_STR");

PRINT_MODULE_USAGE_NAME("l1_adaptive_control", "controller");
PRINT_MODULE_USAGE_COMMAND("start");
PRINT_MODULE_USAGE_COMMAND("status");
PRINT_MODULE_USAGE_COMMAND_DESCR("rc_control", "enable/disable/status optional RC throttle height control");
PRINT_MODULE_USAGE_COMMAND("stop");
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

return 0;
}

extern "C" __EXPORT int l1_adaptive_control_main(int argc, char *argv[])
{
return L1AdaptiveControl::main(argc, argv);
}
