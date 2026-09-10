#include "L1KeyboardThrottleLogic.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>

#include <math.h>
#include <poll.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

extern "C" __EXPORT int l1_keyboard_throttle_main(int argc, char *argv[]);
namespace
{

static constexpr int PUBLISH_INTERVAL_MS = 50;
static constexpr hrt_abstime LANDING_MIN_DISARM_TIME_US = 4'000'000;
static constexpr float LANDING_NEAR_GROUND_M = 0.12f;
static constexpr float LANDING_MAX_VERTICAL_SPEED_M_S = 0.35f;

class RawTerminalGuard
{
public:
	RawTerminalGuard()
	{
		if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &_old_config) == 0) {
			_new_config = _old_config;
			_new_config.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
			_new_config.c_cc[VMIN] = 0;
			_new_config.c_cc[VTIME] = 0;
			_enabled = (tcsetattr(STDIN_FILENO, TCSANOW, &_new_config) == 0);
		}
	}

	~RawTerminalGuard()
	{
		if (_enabled) {
			tcsetattr(STDIN_FILENO, TCSANOW, &_old_config);
		}
	}

private:
	bool _enabled{false};
	struct termios _old_config {};
	struct termios _new_config {};
};

void publish_manual_control(uORB::Publication<manual_control_setpoint_s> &publisher, float throttle)
{
	manual_control_setpoint_s manual{};
	const hrt_abstime now = hrt_absolute_time();

	manual.timestamp = now;
	manual.timestamp_sample = now;
	manual.valid = true;
	manual.data_source = manual_control_setpoint_s::SOURCE_RC;
	manual.roll = 0.f;
	manual.pitch = 0.f;
	manual.yaw = 0.f;
	manual.throttle = throttle;
	manual.flaps = 0.f;
	manual.aux1 = 0.f;
	manual.aux2 = 0.f;
	manual.aux3 = 0.f;
	manual.aux4 = 0.f;
	manual.aux5 = 0.f;
	manual.aux6 = 0.f;
	manual.sticks_moving = (fabsf(throttle) > 0.001f);
	manual.buttons = 0;

	publisher.publish(manual);
}

void publish_arm_disarm_command(uORB::Publication<vehicle_command_s> &publisher, bool arm)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	command.param1 = arm ? 1.f : 0.f;
	publisher.publish(command);
}

void publish_motor_failure_command(uORB::Publication<vehicle_command_s> &publisher, uint8_t failure_type)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE;
	command.param1 = static_cast<float>(vehicle_command_s::FAILURE_UNIT_SYSTEM_MOTOR);
	command.param2 = static_cast<float>(failure_type);
	command.param3 = static_cast<float>(L1_KEYBOARD_THROTTLE_FAILED_MOTOR_INSTANCE);
	publisher.publish(command);
}

int run_keyboard_throttle()
{
	RawTerminalGuard terminal_guard;
	uORB::Publication<manual_control_setpoint_s> manual_control_pub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Subscription vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	L1KeyboardThrottleState state{};
	vehicle_land_detected_s vehicle_land_detected{};
	vehicle_local_position_s vehicle_local_position{};
	bool auto_disarm_after_landing{false};
	bool ground_reference_valid{false};
	float ground_z_ned{0.f};
	hrt_abstime landing_request_time_us{0};

	PX4_INFO("l1_keyboard_throttle started");
	PX4_INFO("keys: 1 takeoff+hover, 2 auto-land, w/s height, x/space hold, 0 fail M1, r restore M1, q quit");
	PX4_INFO("height stick = %.1f", (double)state.throttle);

	while (true) {
		bool command_published_this_cycle{false};

		if (vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_sub.copy(&vehicle_land_detected);
		}

		if (vehicle_local_position_sub.updated()) {
			vehicle_local_position_sub.copy(&vehicle_local_position);
		}

		if (auto_disarm_after_landing) {
			const hrt_abstime now = hrt_absolute_time();
			const bool landing_time_elapsed = landing_request_time_us != 0
				&& now - landing_request_time_us >= LANDING_MIN_DISARM_TIME_US;
			const bool near_captured_ground = ground_reference_valid
				&& vehicle_local_position.z_valid
				&& vehicle_local_position.v_z_valid
				&& PX4_ISFINITE(vehicle_local_position.z)
				&& PX4_ISFINITE(vehicle_local_position.vz)
				&& vehicle_local_position.z >= ground_z_ned - LANDING_NEAR_GROUND_M
				&& fabsf(vehicle_local_position.vz) <= LANDING_MAX_VERTICAL_SPEED_M_S;

			if (vehicle_land_detected.landed || (landing_time_elapsed && near_captured_ground)) {
				publish_arm_disarm_command(vehicle_command_pub, false);
				auto_disarm_after_landing = false;
				landing_request_time_us = 0;
				state.throttle = 0.f;
				publish_manual_control(manual_control_pub, state.throttle);
				command_published_this_cycle = true;
				PX4_INFO("touchdown confirmed: disarm requested");
			}
		}

		struct pollfd fds {};
		fds.fd = STDIN_FILENO;
		fds.events = POLLIN;

		const int poll_ret = poll(&fds, 1, PUBLISH_INTERVAL_MS);

		if (poll_ret > 0 && (fds.revents & POLLIN)) {
			char key = 0;

			while (read(STDIN_FILENO, &key, 1) == 1) {
				const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, key);

				if (action == L1KeyboardThrottleAction::PublishThrottle) {
					PX4_INFO("height stick = %.1f", (double)state.throttle);

				} else if (action == L1KeyboardThrottleAction::TakeoffHover) {
					auto_disarm_after_landing = false;
					landing_request_time_us = 0;

					if (vehicle_local_position.z_valid && PX4_ISFINITE(vehicle_local_position.z)) {
						ground_z_ned = vehicle_local_position.z;
						ground_reference_valid = true;
					}

					publish_arm_disarm_command(vehicle_command_pub, true);
					publish_manual_control(manual_control_pub, L1_KEYBOARD_TAKEOFF_COMMAND_STICK);
					command_published_this_cycle = true;
					PX4_INFO("takeoff requested: arm + climb to hover point");

				} else if (action == L1KeyboardThrottleAction::Land) {
					auto_disarm_after_landing = true;
					landing_request_time_us = hrt_absolute_time();
					publish_manual_control(manual_control_pub, L1_KEYBOARD_LAND_COMMAND_STICK);
					command_published_this_cycle = true;
					PX4_INFO("automatic landing requested");

				} else if (action == L1KeyboardThrottleAction::InjectMotorFailure) {
					publish_motor_failure_command(vehicle_command_pub, vehicle_command_s::FAILURE_TYPE_OFF);
					PX4_WARN("motor %d failure injected", L1_KEYBOARD_THROTTLE_FAILED_MOTOR_INSTANCE);

				} else if (action == L1KeyboardThrottleAction::RestoreMotor) {
					publish_motor_failure_command(vehicle_command_pub, vehicle_command_s::FAILURE_TYPE_OK);
					PX4_WARN("motor %d restored", L1_KEYBOARD_THROTTLE_FAILED_MOTOR_INSTANCE);

				} else if (action == L1KeyboardThrottleAction::Quit) {
					publish_manual_control(manual_control_pub, state.throttle);
					PX4_INFO("l1_keyboard_throttle stopped");
					return 0;
				}
			}
		}

		// Do not overwrite a one-shot 1/2 command in the same cycle. On the next
		// 50 ms cycle normal height-stick publication resumes automatically.
		if (!command_published_this_cycle) {
			publish_manual_control(manual_control_pub, state.throttle);
		}
	}
}

void print_usage()
{
	PX4_INFO("Usage: l1_keyboard_throttle start");
	PX4_INFO("       l1_keyboard_throttle help");
	PX4_INFO("Keys: 1 takeoff+hover, 2 auto-land, w/s height, x/space hold, 0 fail M1, r restore M1, q quit");
}

} // namespace

int l1_keyboard_throttle_main(int argc, char *argv[])
{
	if (argc < 2 || !strcmp(argv[1], "-h") || !strcmp(argv[1], "help")) {
		print_usage();
		return 0;
	}

	if (!strcmp(argv[1], "start")) {
		return run_keyboard_throttle();
	}

	print_usage();
	return 1;
}
