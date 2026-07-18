#include "L1KeyboardThrottleLogic.hpp"

#include <gtest/gtest.h>

TEST(L1KeyboardThrottleLogic, WAndSAdjustThrottleOnly)
{
	L1KeyboardThrottleState state{};

	const L1KeyboardThrottleAction climb = handle_l1_keyboard_throttle_key(state, 'w');
	EXPECT_EQ(climb, L1KeyboardThrottleAction::PublishThrottle);
	EXPECT_FLOAT_EQ(state.throttle, 0.2f);

	const L1KeyboardThrottleAction descend = handle_l1_keyboard_throttle_key(state, 's');
	EXPECT_EQ(descend, L1KeyboardThrottleAction::PublishThrottle);
	EXPECT_FLOAT_EQ(state.throttle, 0.f);
}

TEST(L1KeyboardThrottleLogic, ZeroInjectsFixedMotorFaultWithoutChangingThrottle)
{
	L1KeyboardThrottleState state{};
	state.throttle = 0.4f;

	const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, '0');

	EXPECT_EQ(action, L1KeyboardThrottleAction::InjectMotorFailure);
	EXPECT_EQ(L1_KEYBOARD_THROTTLE_FAILED_MOTOR_INSTANCE, 1);
	EXPECT_FLOAT_EQ(state.throttle, 0.4f);
}

TEST(L1KeyboardThrottleLogic, XAndSpaceZeroThrottleForHeightHold)
{
	L1KeyboardThrottleState state{};
	state.throttle = 0.6f;

	const L1KeyboardThrottleAction x_action = handle_l1_keyboard_throttle_key(state, 'x');
	EXPECT_EQ(x_action, L1KeyboardThrottleAction::PublishThrottle);
	EXPECT_FLOAT_EQ(state.throttle, 0.f);

	state.throttle = -0.4f;
	const L1KeyboardThrottleAction space_action = handle_l1_keyboard_throttle_key(state, ' ');
	EXPECT_EQ(space_action, L1KeyboardThrottleAction::PublishThrottle);
	EXPECT_FLOAT_EQ(state.throttle, 0.f);
}

TEST(L1KeyboardThrottleLogic, RRestoresFixedMotorWithoutChangingThrottle)
{
	L1KeyboardThrottleState state{};
	state.throttle = -0.2f;

	const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, 'r');

	EXPECT_EQ(action, L1KeyboardThrottleAction::RestoreMotor);
	EXPECT_EQ(L1_KEYBOARD_THROTTLE_FAILED_MOTOR_INSTANCE, 1);
	EXPECT_FLOAT_EQ(state.throttle, -0.2f);
}

TEST(L1KeyboardThrottleLogic, QQuitsAndZerosThrottle)
{
	L1KeyboardThrottleState state{};
	state.throttle = 0.6f;

	const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, 'q');

	EXPECT_EQ(action, L1KeyboardThrottleAction::Quit);
	EXPECT_FLOAT_EQ(state.throttle, 0.f);
}
