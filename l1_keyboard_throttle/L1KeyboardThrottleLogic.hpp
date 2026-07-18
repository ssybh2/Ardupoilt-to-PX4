#pragma once

static constexpr int L1_KEYBOARD_THROTTLE_FAILED_MOTOR_INSTANCE = 1;

static constexpr float L1_KEYBOARD_THROTTLE_STEP = 0.2f;
static constexpr float L1_KEYBOARD_THROTTLE_MIN = -1.0f;
static constexpr float L1_KEYBOARD_THROTTLE_MAX = 1.0f;

enum class L1KeyboardThrottleAction {
	None = 0,
	PublishThrottle,
	InjectMotorFailure,
	RestoreMotor,
	Quit
};

struct L1KeyboardThrottleState {
	float throttle{0.f};
};

static inline float constrain_l1_keyboard_throttle(float value)
{
	if (value > L1_KEYBOARD_THROTTLE_MAX) {
		return L1_KEYBOARD_THROTTLE_MAX;
	}

	if (value < L1_KEYBOARD_THROTTLE_MIN) {
		return L1_KEYBOARD_THROTTLE_MIN;
	}

	return value;
}

static inline L1KeyboardThrottleAction handle_l1_keyboard_throttle_key(L1KeyboardThrottleState &state, char key)
{
	if (key == 'w' || key == 'W') {
		state.throttle = constrain_l1_keyboard_throttle(state.throttle + L1_KEYBOARD_THROTTLE_STEP);
		return L1KeyboardThrottleAction::PublishThrottle;
	}

	if (key == 's' || key == 'S') {
		state.throttle = constrain_l1_keyboard_throttle(state.throttle - L1_KEYBOARD_THROTTLE_STEP);
		return L1KeyboardThrottleAction::PublishThrottle;
	}

	if (key == 'x' || key == 'X' || key == ' ') {
		state.throttle = 0.f;
		return L1KeyboardThrottleAction::PublishThrottle;
	}

	if (key == '0') {
		return L1KeyboardThrottleAction::InjectMotorFailure;
	}

	if (key == 'r' || key == 'R') {
		return L1KeyboardThrottleAction::RestoreMotor;
	}

	if (key == 'q' || key == 'Q' || key == 0x03 || key == 0x1b) {
		state.throttle = 0.f;
		return L1KeyboardThrottleAction::Quit;
	}

	return L1KeyboardThrottleAction::None;
}
