#pragma once

#include "GeometricController.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>

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

void Run() override;

void update_subscriptions();
void update_internal_state();
void update_controller_input();
void run_geometric_controller();
void print_debug_info();

uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

vehicle_local_position_s _vehicle_local_position{};
vehicle_attitude_s _vehicle_attitude{};
vehicle_angular_velocity_s _vehicle_angular_velocity{};
vehicle_status_s _vehicle_status{};

bool _has_local_position{false};
bool _has_attitude{false};
bool _has_angular_velocity{false};
bool _has_vehicle_status{false};

InternalState _state{};
bool _state_valid_for_control{false};

GeometricController _geometric_controller{};
GeometricController::Input _controller_input{};
GeometricController::Output _geometric_output{};
bool _geometric_update_executed{false};

perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

hrt_abstime _last_print_us{0};
};
