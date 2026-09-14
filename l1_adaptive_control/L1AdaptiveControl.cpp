#include "L1AdaptiveControl.hpp"

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace
{

float yaw_from_quat_body_to_ned(const float q[4])
{
	return matrix::Eulerf(matrix::Quatf(q)).psi();
}

const char *trajectory_name(uint8_t index)
{
	switch (index) {
	case 1: return "circle_variable_yaw";
	case 2: return "circle_fixed_yaw";
	case 3: return "figure8_fixed_yaw";
	case 4: return "figure8_tilted";
	default: return "hover(source default)";
	}
}

} // namespace

L1AdaptiveControl::L1AdaptiveControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	configure_source_parameters();
}

L1AdaptiveControl::~L1AdaptiveControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void L1AdaptiveControl::configure_source_parameters()
{
	GeometricController::Parameters geometric_parameters{};
	L1AdaptiveAugmentation::Parameters l1_parameters{};
	MotorMixer::Parameters mixer_parameters{};

#if (!REAL_OR_SITL)
	// Original L1Quad SITL constants from mode.h/config.h.
	geometric_parameters.mass_kg = 3.f;
	geometric_parameters.position_gain[0] = 18.f;
	geometric_parameters.position_gain[1] = 18.f;
	geometric_parameters.position_gain[2] = 27.6f;
	geometric_parameters.velocity_gain[0] = 4.f;
	geometric_parameters.velocity_gain[1] = 4.f;
	geometric_parameters.velocity_gain[2] = 6.f;
	geometric_parameters.rotation_gain[0] = 5.4f;
	geometric_parameters.rotation_gain[1] = 5.4f;
	geometric_parameters.rotation_gain[2] = 0.092f;
	geometric_parameters.angular_velocity_gain[0] = 0.6f;
	geometric_parameters.angular_velocity_gain[1] = 0.6f;
	geometric_parameters.angular_velocity_gain[2] = 0.023f;
	geometric_parameters.inertia_kg_m2[0] = 0.023f;
	geometric_parameters.inertia_kg_m2[1] = 0.023f;
	geometric_parameters.inertia_kg_m2[2] = 0.0459f;

	l1_parameters.mass_kg = 3.f;
	l1_parameters.inertia_kg_m2[0] = 0.023f;
	l1_parameters.inertia_kg_m2[1] = 0.023f;
	l1_parameters.inertia_kg_m2[2] = 0.0459f;
	l1_parameters.inertia_inverse[0] = 43.478f;
	l1_parameters.inertia_inverse[1] = 43.478f;
	l1_parameters.inertia_inverse[2] = 21.786f;
	l1_parameters.as_v = -5.f;
	l1_parameters.as_omega = -10.f;
	l1_parameters.cutoff_q1_thrust = 10.f;
	l1_parameters.cutoff_q1_moment = 10.f;
	l1_parameters.cutoff_q2_moment = 2.f;
	l1_parameters.l1_enable = 0; // L1ENABLE_DEFAULT in the source repository.
	mixer_parameters.real_vehicle = false;
#else
	// The default GeometricController and L1 parameter structs contain the
	// uploaded DSun / original REAL_OR_SITL constants.
	l1_parameters.l1_enable = 0; // L1ENABLE_DEFAULT in the source repository.
	mixer_parameters.real_vehicle = true;
#endif

	_geometric_controller.set_parameters(geometric_parameters);
	_l1_adaptive_augmentation.set_parameters(l1_parameters);
	_motor_mixer.set_parameters(mixer_parameters);
	_l1_enabled.store(l1_parameters.l1_enable != 0);
}

bool L1AdaptiveControl::init()
{
	PX4_INFO("L1 source-equivalent adaptive control init (%s)", REAL_OR_SITL ? "REAL" : "SITL");

	// Original L1AdaptiveAugmentation uses dt=0.0025 s.  Run the source pipeline
	// at the matching 400 Hz cadence rather than changing the discrete equations.
	ScheduleOnInterval(2500);
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
	run_motor_mixer();
	publish_motor_commands();

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
		for (int i = 0; i < 4; i++) {
			_state.quat_body_to_ned[i] = _vehicle_attitude.q[i];
		}

		_state.attitude_valid = true;
	}

	if (_has_angular_velocity) {
		for (int i = 0; i < 3; i++) {
			_state.angular_velocity_body[i] = _vehicle_angular_velocity.xyz[i];
		}

		_state.angular_velocity_valid = true;
	}

	_state_valid_for_control = _state.position_valid
				   && _state.velocity_valid
				   && _state.attitude_valid
				   && _state.angular_velocity_valid
				   && !_state.failsafe;
}

void L1AdaptiveControl::update_trajectory_input()
{
	_trajectory_input = TrajectoryGenerator::Input{};
	_trajectory_input.timestamp_us = _state.timestamp_us;

	for (int i = 0; i < 3; i++) {
		_trajectory_input.current_position_ned[i] = _state.position_ned[i];
		_trajectory_input.current_velocity_ned[i] = _state.velocity_ned[i];
	}

	_trajectory_input.current_yaw = yaw_from_quat_body_to_ned(_state.quat_body_to_ned);
	_trajectory_input.land_flag = _land_flag.load();
	_trajectory_input.state_valid_for_control = _state_valid_for_control;
	_trajectory_input.armed = _state.armed;
	_trajectory_input.failsafe = _state.failsafe;

	TrajectoryGenerator::Parameters trajectory_parameters = _trajectory_generator.parameters();
	trajectory_parameters.trajectory_index = _trajectory_index.load();
	_trajectory_generator.set_parameters(trajectory_parameters);
}

void L1AdaptiveControl::run_trajectory_generator()
{
	_trajectory_update_executed = _trajectory_generator.update(_trajectory_input, _trajectory_output);
}

void L1AdaptiveControl::update_controller_input()
{
	_controller_input = GeometricController::Input{};
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

	for (int i = 0; i < 2; i++) {
		_controller_input.target_yaw[i] = _trajectory_output.yaw[i];
		_controller_input.target_yaw_dot[i] = _trajectory_output.yaw_dot[i];
		_controller_input.target_yaw_ddot[i] = _trajectory_output.yaw_ddot[i];
	}

	_controller_input.state_valid_for_control = _state_valid_for_control && _trajectory_output.valid;
	_controller_input.armed = _state.armed;
	_controller_input.failsafe = _state.failsafe;
	_controller_input.nav_state = _state.nav_state;
}

void L1AdaptiveControl::run_geometric_controller()
{
	_geometric_update_executed = _geometric_controller.update(_controller_input, _geometric_output);

	_baseline_thrust_moment[0] = _geometric_output.target_thrust;
	_baseline_thrust_moment[1] = _geometric_output.M[0];
	_baseline_thrust_moment[2] = _geometric_output.M[1];
	_baseline_thrust_moment[3] = _geometric_output.M[2];
}

void L1AdaptiveControl::run_l1_adaptive_augmentation()
{
	L1AdaptiveAugmentation::Input l1_input{};
	l1_input.timestamp_us = _state.timestamp_us;
	l1_input.baseline_valid = _geometric_output.valid;
	l1_input.state_valid = _state_valid_for_control;
	l1_input.armed = _state.armed;
	l1_input.failsafe = _state.failsafe;

	for (int i = 0; i < 3; i++) {
		l1_input.velocity_ned[i] = _state.velocity_ned[i];
		l1_input.angular_velocity_body[i] = _state.angular_velocity_body[i];
	}

	for (int i = 0; i < 4; i++) {
		l1_input.quat_body_to_ned[i] = _state.quat_body_to_ned[i];
		l1_input.baseline_thrust_moment[i] = _baseline_thrust_moment[i];
	}

	_l1_update_executed = _l1_adaptive_augmentation.update(l1_input, _l1_output);

	for (int i = 0; i < 4; i++) {
		// Exactly the source ModeAdaptive::run() combination order.
		_combined_thrust_moment[i] = _baseline_thrust_moment[i] + _l1_output.adaptive_thrust_moment[i];
	}
}

void L1AdaptiveControl::run_motor_mixer()
{
	_motor_mix_executed = false;
	_motor_command = MotorMixer::MotorCommand{};

	if (!_state_valid_for_control || !_state.armed || _state.failsafe
	    || !_geometric_output.valid || !_l1_update_executed) {
		return;
	}

	MotorMixer::ThrustMoment thrust_moment_cmd{};

	for (int i = 0; i < 4; i++) {
		thrust_moment_cmd(i) = _combined_thrust_moment[i];
	}

	_motor_mix_executed = _motor_mixer.mix(thrust_moment_cmd, _motor_command);

	if (!_motor_mix_executed) {
		return;
	}

	// Preserve the source motorPWM saturation exactly.
	for (int i = 0; i < 4; i++) {
		if (_motor_command(i) < 0.f) {
			_motor_command(i) = 0.f;

		} else if (_motor_command(i) > 100.f) {
			_motor_command(i) = 100.f;
		}
	}

	// Preserve the source landing-complete override.
	if (_trajectory_output.landing_complete) {
		for (int i = 0; i < 4; i++) {
			_motor_command(i) = 1.f;
		}
	}
}

void L1AdaptiveControl::publish_motor_commands()
{
	_actuator_motors_published = false;
	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	actuator_motors.reversible_flags = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	if (!_motor_mix_executed || !_state.armed || _state.failsafe) {
		_actuator_motors_pub.publish(actuator_motors);
		return;
	}

	for (int i = 0; i < 4; i++) {
		// Source rc_write maps 0..100 to PWM 1000..2000.  PX4 actuator_motors
		// represents the same final motor command as normalized 0..1 thrust.
		_published_motor_control[i] = _motor_command(i) * 0.01f;
		actuator_motors.control[i] = _published_motor_control[i];
	}

	_actuator_motors_pub.publish(actuator_motors);
	_actuator_motors_published = true;
	_actuator_publish_count++;
}

void L1AdaptiveControl::print_debug_info()
{
	PX4_INFO("L1src | armed=%d failsafe=%d valid=%d traj=%u t=%.2f l1=%d",
		 (int)_state.armed, (int)_state.failsafe, (int)_state_valid_for_control,
		 (unsigned)_trajectory_index.load(), (double)_trajectory_output.time_in_this_run_s,
		 (int)_l1_enabled.load());
	PX4_INFO("  ub=[%.2f %.3f %.3f %.3f] uad=[%.2f %.3f %.3f %.3f]",
		 (double)_baseline_thrust_moment[0], (double)_baseline_thrust_moment[1],
		 (double)_baseline_thrust_moment[2], (double)_baseline_thrust_moment[3],
		 (double)_l1_output.adaptive_thrust_moment[0], (double)_l1_output.adaptive_thrust_moment[1],
		 (double)_l1_output.adaptive_thrust_moment[2], (double)_l1_output.adaptive_thrust_moment[3]);
	PX4_INFO("  motor%%=[%.2f %.2f %.2f %.2f] publish=%d count=%u",
		 (double)_motor_command(0), (double)_motor_command(1),
		 (double)_motor_command(2), (double)_motor_command(3),
		 (int)_actuator_motors_published, (unsigned)_actuator_publish_count);
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
	PX4_INFO("L1 source-equivalent adaptive control (%s)", REAL_OR_SITL ? "REAL" : "SITL");
	PX4_INFO("  state: armed=%d failsafe=%d nav=%u valid=%d",
		 (int)_state.armed, (int)_state.failsafe, (unsigned)_state.nav_state,
		 (int)_state_valid_for_control);
	PX4_INFO("  trajectory: index=%u (%s) valid=%d t=%.3f land=%d/%d",
		 (unsigned)_trajectory_index.load(), trajectory_name(_trajectory_index.load()),
		 (int)_trajectory_output.valid, (double)_trajectory_output.time_in_this_run_s,
		 (int)_trajectory_output.landing_triggered, (int)_trajectory_output.landing_complete);
	PX4_INFO("  pipeline: trajectory=%d geometric=%d l1=%d mixer=%d actuator=%d",
		 (int)_trajectory_update_executed, (int)_geometric_update_executed,
		 (int)_l1_update_executed, (int)_motor_mix_executed, (int)_actuator_motors_published);
	PX4_INFO("  L1 enable=%d motor%%=[%.2f %.2f %.2f %.2f]",
		 (int)_l1_enabled.load(), (double)_motor_command(0), (double)_motor_command(1),
		 (double)_motor_command(2), (double)_motor_command(3));

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

void L1AdaptiveControl::set_trajectory_index(uint8_t trajectory_index)
{
	_trajectory_index.store(trajectory_index);
}

void L1AdaptiveControl::set_l1_enabled(bool enabled)
{
	L1AdaptiveAugmentation::Parameters parameters = _l1_adaptive_augmentation.parameters();
	parameters.l1_enable = enabled ? 1 : 0;
	_l1_adaptive_augmentation.set_parameters(parameters);
	_l1_enabled.store(enabled);
}

int L1AdaptiveControl::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_ERR("module not running");
		return -1;
	}

	L1AdaptiveControl *instance = get_instance();

	if (instance == nullptr) {
		PX4_ERR("module instance unavailable");
		return -1;
	}

	if (argc >= 1 && !strcmp(argv[0], "trajectory")) {
		if (argc < 2 || !strcmp(argv[1], "status")) {
			PX4_INFO("trajectory %u: %s", (unsigned)instance->trajectory_index(),
				 trajectory_name(instance->trajectory_index()));
			return 0;
		}

		const int index = atoi(argv[1]);

		if (index < 0 || index > 4) {
			return print_usage("trajectory index must be 0..4");
		}

		instance->set_trajectory_index(static_cast<uint8_t>(index));
		PX4_INFO("trajectory %d: %s", index, trajectory_name(static_cast<uint8_t>(index)));
		return 0;
	}

	if (argc >= 1 && !strcmp(argv[0], "l1")) {
		if (argc < 2 || !strcmp(argv[1], "status")) {
			PX4_INFO("L1 augmentation: %s", instance->l1_enabled() ? "enabled" : "disabled");
			return 0;
		}

		if (!strcmp(argv[1], "enable") || !strcmp(argv[1], "1")) {
			instance->set_l1_enabled(true);
			return 0;
		}

		if (!strcmp(argv[1], "disable") || !strcmp(argv[1], "0")) {
			instance->set_l1_enabled(false);
			return 0;
		}

		return print_usage("l1 expects enable/disable/status");
	}

	if (argc >= 1 && !strcmp(argv[0], "land")) {
		if (argc < 2 || !strcmp(argv[1], "status")) {
			PX4_INFO("LandFlag: %s", instance->land_flag() ? "1" : "0");
			return 0;
		}

		if (!strcmp(argv[1], "enable") || !strcmp(argv[1], "1")) {
			instance->set_land_flag(true);
			return 0;
		}

		if (!strcmp(argv[1], "disable") || !strcmp(argv[1], "0")) {
			instance->set_land_flag(false);
			return 0;
		}

		return print_usage("land expects enable/disable/status");
	}

	return print_usage("unknown command");
}

int L1AdaptiveControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
R"DESCR_STR(
### Description
Source-equivalent modular port of sigma-pi/L1Quad ModeAdaptive.

Numerical pipeline is preserved as:
ACRL trajectory -> geometricController -> L1AdaptiveAugmentation ->
(u_baseline + u_adaptive) -> original iterative motorMixing -> actuator_motors.

REAL_OR_SITL defaults to 0, matching the original repository.  L1ENABLE also
starts at the original default 0; use `l1_adaptive_control l1 enable` to enable
adaptive augmentation.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("l1_adaptive_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("trajectory", "0..4 or status (original TRAJINDEX semantics)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("l1", "enable/disable/status (original L1ENABLE semantics)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("land", "enable/disable/status (original LandFlag semantics)");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int l1_adaptive_control_main(int argc, char *argv[])
{
	return L1AdaptiveControl::main(argc, argv);
}
