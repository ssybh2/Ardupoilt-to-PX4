#include "L1AdaptiveControl.hpp"

L1AdaptiveControl::L1AdaptiveControl() :
ModuleParams(nullptr),
ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
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

// 10 Hz debug loop.
// Current stage:
// - subscribe uORB
// - convert uORB into internal state
// - generate placeholder trajectory
// - convert trajectory + state into GeometricController input
// - call empty GeometricController framework
ScheduleOnInterval(100_ms);

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

// Stage 4 placeholder:
// We do not compute yaw from quaternion yet.
// Later this should be extracted from quat_body_to_ned.
_trajectory_input.current_yaw = 0.f;

_trajectory_input.state_valid_for_control = _state_valid_for_control;
_trajectory_input.armed = _state.armed;
_trajectory_input.failsafe = _state.failsafe;
_trajectory_input.nav_state = _state.nav_state;
}

void L1AdaptiveControl::run_trajectory_generator()
{
_trajectory_update_executed = _trajectory_generator.update(_trajectory_input, _trajectory_output);
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

void L1AdaptiveControl::print_debug_info()
{
PX4_INFO("========== L1 adaptive trajectory framework ==========");

PX4_INFO("subscriptions: local_pos=%d attitude=%d angular_vel=%d status=%d",
 (int)_has_local_position,
 (int)_has_attitude,
 (int)_has_angular_velocity,
 (int)_has_vehicle_status);

PX4_INFO("state flags: pos_valid=%d vel_valid=%d",
 (int)_state.position_valid,
 (int)_state.velocity_valid);

PX4_INFO("state flags: att_valid=%d omega_valid=%d valid_for_control=%d",
 (int)_state.attitude_valid,
 (int)_state.angular_velocity_valid,
 (int)_state_valid_for_control);

PX4_INFO("status: armed=%d arming_state=%u nav_state=%u failsafe=%d",
 (int)_state.armed,
 (unsigned)_state.arming_state,
 (unsigned)_state.nav_state,
 (int)_state.failsafe);

PX4_INFO("state.position_ned [m]: x=%.3f y=%.3f z=%.3f",
 (double)_state.position_ned[0],
 (double)_state.position_ned[1],
 (double)_state.position_ned[2]);

PX4_INFO("state.velocity_ned [m/s]: vx=%.3f vy=%.3f vz=%.3f",
 (double)_state.velocity_ned[0],
 (double)_state.velocity_ned[1],
 (double)_state.velocity_ned[2]);

PX4_INFO("trajectory: update_executed=%d output_valid=%d mode=%u elapsed=%.2f s",
		 (int)_trajectory_update_executed,
		 (int)_trajectory_output.valid,
		 (unsigned)_trajectory_output.mode,
		 (double)_trajectory_output.elapsed_time_s);

PX4_INFO("trajectory target_pos_ned [m]: x=%.3f y=%.3f z=%.3f",
 (double)_trajectory_output.position_ned[0],
 (double)_trajectory_output.position_ned[1],
 (double)_trajectory_output.position_ned[2]);

PX4_INFO("trajectory target_vel_ned [m/s]: vx=%.3f vy=%.3f vz=%.3f",
 (double)_trajectory_output.velocity_ned[0],
 (double)_trajectory_output.velocity_ned[1],
 (double)_trajectory_output.velocity_ned[2]);

PX4_INFO("trajectory target_acc_ned [m/s^2]: ax=%.3f ay=%.3f az=%.3f",
 (double)_trajectory_output.acceleration_ned[0],
 (double)_trajectory_output.acceleration_ned[1],
 (double)_trajectory_output.acceleration_ned[2]);

PX4_INFO("trajectory yaw: yaw=%.3f yaw_rate=%.3f yaw_accel=%.3f",
 (double)_trajectory_output.yaw,
 (double)_trajectory_output.yaw_rate,
 (double)_trajectory_output.yaw_accel);

PX4_INFO("geometric placeholder: update_executed=%d output_valid=%d",
 (int)_geometric_update_executed,
 (int)_geometric_output.valid);

PX4_INFO("geometric errors: ep=[%.3f %.3f %.3f] ev=[%.3f %.3f %.3f]",
		 (double)_geometric_output.position_error_ned[0],
		 (double)_geometric_output.position_error_ned[1],
		 (double)_geometric_output.position_error_ned[2],
		 (double)_geometric_output.velocity_error_ned[0],
		 (double)_geometric_output.velocity_error_ned[1],
		 (double)_geometric_output.velocity_error_ned[2]);

	PX4_INFO("geometric force_ned=[%.3f %.3f %.3f] body_z_ned=[%.3f %.3f %.3f]",
		 (double)_geometric_output.target_force_ned[0],
		 (double)_geometric_output.target_force_ned[1],
		 (double)_geometric_output.target_force_ned[2],
		 (double)_geometric_output.body_z_axis_ned[0],
		 (double)_geometric_output.body_z_axis_ned[1],
		 (double)_geometric_output.body_z_axis_ned[2]);

	PX4_INFO("geometric force derivatives: Fdot=[%.3f %.3f %.3f] Fddot=[%.3f %.3f %.3f]",
		 (double)_geometric_output.target_force_dot_ned[0],
		 (double)_geometric_output.target_force_dot_ned[1],
		 (double)_geometric_output.target_force_dot_ned[2],
		 (double)_geometric_output.target_force_ddot_ned[0],
		 (double)_geometric_output.target_force_ddot_ned[1],
		 (double)_geometric_output.target_force_ddot_ned[2]);

	PX4_INFO("geometric acc/jerk error: ae=[%.3f %.3f %.3f] je=[%.3f %.3f %.3f]",
		 (double)_geometric_output.acceleration_error_ned[0],
		 (double)_geometric_output.acceleration_error_ned[1],
		 (double)_geometric_output.acceleration_error_ned[2],
		 (double)_geometric_output.jerk_error_ned[0],
		 (double)_geometric_output.jerk_error_ned[1],
		 (double)_geometric_output.jerk_error_ned[2]);

	PX4_INFO("geometric b3c: [%.3f %.3f %.3f]",
		 (double)_geometric_output.b3c_ned[0],
		 (double)_geometric_output.b3c_ned[1],
		 (double)_geometric_output.b3c_ned[2]);

	PX4_INFO("geometric b3c_dot: [%.3f %.3f %.3f]",
		 (double)_geometric_output.b3c_dot_ned[0],
		 (double)_geometric_output.b3c_dot_ned[1],
		 (double)_geometric_output.b3c_dot_ned[2]);

	PX4_INFO("geometric b3c_ddot: [%.3f %.3f %.3f]",
		 (double)_geometric_output.b3c_ddot_ned[0],
		 (double)_geometric_output.b3c_ddot_ned[1],
		 (double)_geometric_output.b3c_ddot_ned[2]);

	PX4_INFO("geometric A2: [%.3f %.3f %.3f]",
		 (double)_geometric_output.a2_ned[0],
		 (double)_geometric_output.a2_ned[1],
		 (double)_geometric_output.a2_ned[2]);

	PX4_INFO("geometric A2_dot: [%.3f %.3f %.3f]",
		 (double)_geometric_output.a2_dot_ned[0],
		 (double)_geometric_output.a2_dot_ned[1],
		 (double)_geometric_output.a2_dot_ned[2]);

	PX4_INFO("geometric A2_ddot: [%.3f %.3f %.3f]",
		 (double)_geometric_output.a2_ddot_ned[0],
		 (double)_geometric_output.a2_ddot_ned[1],
		 (double)_geometric_output.a2_ddot_ned[2]);

	PX4_INFO("geometric b2c: [%.3f %.3f %.3f]",
		 (double)_geometric_output.b2c_ned[0],
		 (double)_geometric_output.b2c_ned[1],
		 (double)_geometric_output.b2c_ned[2]);

	PX4_INFO("geometric b2c_dot: [%.3f %.3f %.3f]",
		 (double)_geometric_output.b2c_dot_ned[0],
		 (double)_geometric_output.b2c_dot_ned[1],
		 (double)_geometric_output.b2c_dot_ned[2]);

	PX4_INFO("geometric b2c_ddot: [%.3f %.3f %.3f]",
		 (double)_geometric_output.b2c_ddot_ned[0],
		 (double)_geometric_output.b2c_ddot_ned[1],
		 (double)_geometric_output.b2c_ddot_ned[2]);

	PX4_INFO("geometric desired axes: xd=[%.3f %.3f %.3f] yd=[%.3f %.3f %.3f]",
		 (double)_geometric_output.desired_body_x_axis_ned[0],
		 (double)_geometric_output.desired_body_x_axis_ned[1],
		 (double)_geometric_output.desired_body_x_axis_ned[2],
		 (double)_geometric_output.desired_body_y_axis_ned[0],
		 (double)_geometric_output.desired_body_y_axis_ned[1],
		 (double)_geometric_output.desired_body_y_axis_ned[2]);

	PX4_INFO("geometric desired z: zd=[%.3f %.3f %.3f] eR=[%.3f %.3f %.3f]",
		 (double)_geometric_output.desired_body_z_axis_ned[0],
		 (double)_geometric_output.desired_body_z_axis_ned[1],
		 (double)_geometric_output.desired_body_z_axis_ned[2],
		 (double)_geometric_output.rotation_error[0],
		 (double)_geometric_output.rotation_error[1],
		 (double)_geometric_output.rotation_error[2]);

	PX4_INFO("geometric angular error: ew=[%.4f %.4f %.4f] omega_d=[%.4f %.4f %.4f]",
		 (double)_geometric_output.angular_velocity_error[0],
		 (double)_geometric_output.angular_velocity_error[1],
		 (double)_geometric_output.angular_velocity_error[2],
		 (double)_geometric_output.desired_angular_velocity_body[0],
		 (double)_geometric_output.desired_angular_velocity_body[1],
		 (double)_geometric_output.desired_angular_velocity_body[2]);

	PX4_INFO("geometric moment split: Mpd=[%.5f %.5f %.5f] Mgyro=[%.5f %.5f %.5f]",
		 (double)_geometric_output.pd_moment_newton_meter[0],
		 (double)_geometric_output.pd_moment_newton_meter[1],
		 (double)_geometric_output.pd_moment_newton_meter[2],
		 (double)_geometric_output.gyro_moment_newton_meter[0],
		 (double)_geometric_output.gyro_moment_newton_meter[1],
		 (double)_geometric_output.gyro_moment_newton_meter[2]);

	PX4_INFO("geometric inertia: Jomega=[%.5f %.5f %.5f]",
		 (double)_geometric_output.j_omega_body[0],
		 (double)_geometric_output.j_omega_body[1],
		 (double)_geometric_output.j_omega_body[2]);

	PX4_INFO("geometric basic output: F=%.3f N M=[%.3f %.3f %.3f] Nm",
		 (double)_geometric_output.thrust_newton,
		 (double)_geometric_output.moment_newton_meter[0],
		 (double)_geometric_output.moment_newton_meter[1],
		 (double)_geometric_output.moment_newton_meter[2]);
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
PX4_INFO("L1 adaptive control is running");

PX4_INFO("has local_position=%d attitude=%d angular_velocity=%d vehicle_status=%d",
 (int)_has_local_position,
 (int)_has_attitude,
 (int)_has_angular_velocity,
 (int)_has_vehicle_status);

PX4_INFO("state_valid_for_control=%d trajectory_update=%d trajectory_valid=%d",
 (int)_state_valid_for_control,
 (int)_trajectory_update_executed,
 (int)_trajectory_output.valid);

PX4_INFO("geometric_update_executed=%d geometric_output_valid=%d",
 (int)_geometric_update_executed,
 (int)_geometric_output.valid);

perf_print_counter(_loop_perf);
perf_print_counter(_loop_interval_perf);

return 0;
}

int L1AdaptiveControl::custom_command(int argc, char *argv[])
{
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
- Subscribe vehicle_status
- Convert uORB messages into internal controller state
- Generate placeholder hold-position trajectory
- Convert trajectory output into GeometricController input
- Call empty GeometricController framework
- Print debug state only
- No control output is published
)DESCR_STR");

PRINT_MODULE_USAGE_NAME("l1_adaptive_control", "controller");
PRINT_MODULE_USAGE_COMMAND("start");
PRINT_MODULE_USAGE_COMMAND("status");
PRINT_MODULE_USAGE_COMMAND("stop");
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

return 0;
}

extern "C" __EXPORT int l1_adaptive_control_main(int argc, char *argv[])
{
return L1AdaptiveControl::main(argc, argv);
}
