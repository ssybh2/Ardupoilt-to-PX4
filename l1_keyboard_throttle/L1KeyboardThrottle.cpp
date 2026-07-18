#include "L1KeyboardThrottleLogic.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>

#include <math.h>
#include <poll.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

extern "C" __EXPORT int l1_keyboard_throttle_main(int argc, char *argv[]);
namespace
{

static constexpr int PUBLISH_INTERVAL_MS = 50;

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
	L1KeyboardThrottleState state{};

	PX4_INFO("l1_keyboard_throttle started");
	PX4_INFO("keys: w +0.2, s -0.2, x/space hold height, 0 fail motor 1, r restore motor 1, q quit");
	PX4_INFO("throttle = %.1f", (double)state.throttle);

	while (true) {
		struct pollfd fds {};
		fds.fd = STDIN_FILENO;
		fds.events = POLLIN;

		const int poll_ret = poll(&fds, 1, PUBLISH_INTERVAL_MS);

		if (poll_ret > 0 && (fds.revents & POLLIN)) {
			char key = 0;

			while (read(STDIN_FILENO, &key, 1) == 1) {
				const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, key);

				if (action == L1KeyboardThrottleAction::PublishThrottle) {
					PX4_INFO("throttle = %.1f", (double)state.throttle);

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

		publish_manual_control(manual_control_pub, state.throttle);
	}
}

void print_usage()
{
	PX4_INFO("Usage: l1_keyboard_throttle start");
	PX4_INFO("       l1_keyboard_throttle help");
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
