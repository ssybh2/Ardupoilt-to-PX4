#pragma once

#include "GeometricController.hpp"
#include "TrajectoryGenerator.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class L1AdaptiveControl :
public ModuleBase<L1AdaptiveControl>,
public ModuleParams,
public px4::ScheduledWorkItem
{
public:
L1AdaptiveControl();
~L1AdaptiveControl() override;

static int task_spawn(int argc, char *argv[]);
static int custom_command(int argc, char *argv[]);
static int print_usage(const char *reason = nullptr);

bool init();

int print_status() override;

void set_rc_height_control_enabled(bool enabled);
bool rc_height_control_enabled() const { return _rc_height_control_enabled.load(); }
void set_trajectory_mode(TrajectoryGenerator::CommandedMode mode);
TrajectoryGenerator::CommandedMode trajectory_mode() const
{
return static_cast<TrajectoryGenerator::CommandedMode>(_trajectory_command_mode.load());
}

private:
struct InternalState {
hrt_abstime timestamp_us{0};

float position_ned[3]{0.f, 0.f, 0.f};
float velocity_ned[3]{0.f, 0.f, 0.f};

float quat_body_to_ned[4]{1.f, 0.f, 0.f, 0.f};
float angular_velocity_body[3]{0.f, 0.f, 0.f};

bool position_valid{false};
bool velocity_valid{false};
bool attitude_valid{false};
bool angular_velocity_valid{false};

bool armed{false};
bool failsafe{false};
uint8_t arming_state{0};
uint8_t nav_state{0};
};

struct L1AdaptiveState {
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

void Run() override;

void update_subscriptions();
void update_internal_state();

void update_trajectory_input();
void run_trajectory_generator();
void update_manual_height_control_input();
void apply_trajectory_command();

void update_controller_input();
void run_geometric_controller();

void reset_l1_adaptive_state();
void run_l1_adaptive_augmentation();
void publish_control_setpoints();

void print_debug_info();

uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

vehicle_local_position_s _vehicle_local_position{};
vehicle_attitude_s _vehicle_attitude{};
vehicle_angular_velocity_s _vehicle_angular_velocity{};
manual_control_setpoint_s _manual_control_setpoint{};
vehicle_status_s _vehicle_status{};

bool _has_local_position{false};
bool _has_attitude{false};
bool _has_angular_velocity{false};
bool _has_manual_control_setpoint{false};
bool _has_vehicle_status{false};

px4::atomic_bool _rc_height_control_enabled{false};
px4::atomic<uint8_t> _trajectory_command_mode{static_cast<uint8_t>(TrajectoryGenerator::CommandedMode::Hover)};
bool _manual_height_control_valid{false};
float _manual_height_stick{0.f};

InternalState _state{};
bool _state_valid_for_control{false};

TrajectoryGenerator _trajectory_generator{};
TrajectoryGenerator::Input _trajectory_input{};
TrajectoryGenerator::Output _trajectory_output{};
bool _trajectory_update_executed{false};

GeometricController _geometric_controller{};
GeometricController::Input _controller_input{};
GeometricController::Output _geometric_output{};
bool _geometric_update_executed{false};

L1AdaptiveState _l1_state{};
float _l1_output_thrust_moment[4]{0.f, 0.f, 0.f, 0.f};
float _combined_thrust_moment[4]{0.f, 0.f, 0.f, 0.f};
bool _l1_update_executed{false};

float _published_thrust_body[3]{0.f, 0.f, 0.f};
float _published_torque_body[3]{0.f, 0.f, 0.f};
bool _control_setpoint_published{false};
uint32_t _control_setpoint_publish_count{0};

perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

hrt_abstime _last_print_us{0};
};
