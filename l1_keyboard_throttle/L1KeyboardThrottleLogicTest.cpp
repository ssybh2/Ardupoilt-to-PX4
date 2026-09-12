#include "L1KeyboardThrottleLogic.hpp"

#include <gtest/gtest.h>

TEST(L1KeyboardThrottleLogic, UsesNormalDisarmWhenLandDetectorConfirmsTouchdown)
{
	EXPECT_EQ(decide_l1_keyboard_disarm(true, true, true), L1KeyboardDisarmAction::Normal);
}

TEST(L1KeyboardThrottleLogic, UsesForcedDisarmForTimedNearGroundFallback)
{
	EXPECT_EQ(decide_l1_keyboard_disarm(false, true, true), L1KeyboardDisarmAction::Force);
	EXPECT_EQ(decide_l1_keyboard_disarm(false, false, true), L1KeyboardDisarmAction::None);
	EXPECT_EQ(decide_l1_keyboard_disarm(false, true, false), L1KeyboardDisarmAction::None);
}

TEST(L1KeyboardThrottleLogic, OneRequestsTakeoffAndNeutralizesHeightStick)
{
	L1KeyboardThrottleState state{};
	state.throttle = 0.6f;

	const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, '1');

	EXPECT_EQ(action, L1KeyboardThrottleAction::TakeoffHover);
	EXPECT_FLOAT_EQ(state.throttle, 0.f);
	EXPECT_FLOAT_EQ(L1_KEYBOARD_TAKEOFF_COMMAND_STICK, 0.97f);
	EXPECT_TRUE(state.takeoff_pending);
	EXPECT_FALSE(consume_l1_keyboard_takeoff_command(state, false));
	EXPECT_TRUE(state.takeoff_pending);
	EXPECT_TRUE(consume_l1_keyboard_takeoff_command(state, true));
	EXPECT_FALSE(state.takeoff_pending);
}

TEST(L1KeyboardThrottleLogic, TwoRequestsLandingAndNeutralizesHeightStick)
{
	L1KeyboardThrottleState state{};
	state.throttle = -0.4f;

	const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, '2');

	EXPECT_EQ(action, L1KeyboardThrottleAction::Land);
	EXPECT_FLOAT_EQ(state.throttle, 0.f);
	EXPECT_FLOAT_EQ(L1_KEYBOARD_LAND_COMMAND_STICK, -0.97f);
}

TEST(L1KeyboardThrottleLogic, WAndSAdjustHeightStickOnly)
{
	L1KeyboardThrottleState state{};

	const L1KeyboardThrottleAction climb = handle_l1_keyboard_throttle_key(state, 'w');
	EXPECT_EQ(climb, L1KeyboardThrottleAction::PublishThrottle);
	EXPECT_FLOAT_EQ(state.throttle, 0.2f);

	const L1KeyboardThrottleAction descend = handle_l1_keyboard_throttle_key(state, 's');
	EXPECT_EQ(descend, L1KeyboardThrottleAction::PublishThrottle);
	EXPECT_FLOAT_EQ(state.throttle, 0.f);
}

TEST(L1KeyboardThrottleLogic, ZeroInjectsFixedMotorFaultWithoutChangingHeightStick)
{
	L1KeyboardThrottleState state{};
	state.throttle = 0.4f;

	const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, '0');

	EXPECT_EQ(action, L1KeyboardThrottleAction::InjectMotorFailure);
	EXPECT_EQ(L1_KEYBOARD_THROTTLE_FAILED_MOTOR_INSTANCE, 1);
	EXPECT_FLOAT_EQ(state.throttle, 0.4f);
}

TEST(L1KeyboardThrottleLogic, XAndSpaceZeroHeightStick)
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

TEST(L1KeyboardThrottleLogic, RRestoresFixedMotorWithoutChangingHeightStick)
{
	L1KeyboardThrottleState state{};
	state.throttle = -0.2f;

	const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, 'r');

	EXPECT_EQ(action, L1KeyboardThrottleAction::RestoreMotor);
	EXPECT_EQ(L1_KEYBOARD_THROTTLE_FAILED_MOTOR_INSTANCE, 1);
	EXPECT_FLOAT_EQ(state.throttle, -0.2f);
}

TEST(L1KeyboardThrottleLogic, QQuitsAndZerosHeightStick)
{
	L1KeyboardThrottleState state{};
	state.throttle = 0.6f;

	const L1KeyboardThrottleAction action = handle_l1_keyboard_throttle_key(state, 'q');

	EXPECT_EQ(action, L1KeyboardThrottleAction::Quit);
	EXPECT_FLOAT_EQ(state.throttle, 0.f);
}
