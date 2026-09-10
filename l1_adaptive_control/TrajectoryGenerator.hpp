#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

class TrajectoryGenerator
{
public:
	enum class Mode : uint8_t {
		WaitForValidState = 0,
		Takeoff = 1,
		Hover = 2,
		Landing = 3,
		Landed = 4
	};

	// The L1 wrapper still exposes the historical commanded-mode interface.
	// This branch no longer generates a circle trajectory: both values resolve
	// to hover behavior. Keyboard 1/2 are handled through the manual-control
	// command sentinels below.
	enum class CommandedMode : uint8_t {
		Hover = 0,
		Circle = 1 // legacy compatibility only; no circle is generated
	};

	struct Input {
		hrt_abstime timestamp_us{0};

		float current_position_ned[3]{0.f, 0.f, 0.f};
		float current_yaw{0.f};

		bool state_valid_for_control{false};
		bool armed{false};
		bool failsafe{false};
		uint8_t nav_state{0};

		float manual_height_stick{0.f};
		bool manual_height_control_enabled{false};
		bool manual_height_control_valid{false};
	};

	struct Output {
		hrt_abstime timestamp_us{0};

		float position_ned[3]{0.f, 0.f, 0.f};
		float velocity_ned[3]{0.f, 0.f, 0.f};
		float acceleration_ned[3]{0.f, 0.f, 0.f};
		float jerk_ned[3]{0.f, 0.f, 0.f};
		float snap_ned[3]{0.f, 0.f, 0.f};

		float yaw{0.f};
		float yaw_rate{0.f};
		float yaw_accel{0.f};

		float elapsed_time_s{0.f};
		Mode mode{Mode::WaitForValidState};
		bool valid{false};
	};

	TrajectoryGenerator() = default;
	~TrajectoryGenerator() = default;

	bool update(const Input &input, Output &output);

	void reset();
	void set_commanded_mode(CommandedMode mode);

	CommandedMode commanded_mode() const { return _commanded_mode; }
	float takeoff_height_m() const { return TAKEOFF_HEIGHT_M; }
	float takeoff_duration_s() const { return TAKEOFF_DURATION_S; }
	float landing_duration_s() const { return LANDING_DURATION_S; }

	static constexpr float KEYBOARD_TAKEOFF_COMMAND_STICK = 0.97f;
	static constexpr float KEYBOARD_LAND_COMMAND_STICK = -0.97f;

	const Input &last_input() const { return _last_input; }
	const Output &last_output() const { return _last_output; }

private:
	void handle_keyboard_command(const Input &input);
	void initialize_takeoff(const Input &input);
	void initialize_landing(const Input &input);
	void set_zero_derivatives(Output &output);
	void set_hold_position(Output &output, const float position_ned[3]);
	void update_manual_hold_target(const Input &input, Output &output);
	float update_manual_height_reference(const Input &input);
	void update_takeoff_target(const Input &input, Output &output);
	void update_hover_target(const Input &input, Output &output);
	void update_landing_target(const Input &input, Output &output);
	void sync_hover_reference_from_output(const Output &output);

	bool _initialized{false};
	bool _takeoff_requested{false};
	bool _landing_requested{false};
	bool _landing_initialized{false};

	hrt_abstime _takeoff_start_time_us{0};
	hrt_abstime _landing_start_time_us{0};
	hrt_abstime _last_update_us{0};

	float _ground_position_ned[3]{0.f, 0.f, 0.f};
	float _takeoff_start_position_ned[3]{0.f, 0.f, 0.f};
	float _takeoff_target_position_ned[3]{0.f, 0.f, -1.f};
	float _hover_position_ned[3]{0.f, 0.f, -1.f};
	float _manual_hold_position_ned[3]{0.f, 0.f, 0.f};
	float _landing_start_position_ned[3]{0.f, 0.f, 0.f};
	float _landing_target_position_ned[3]{0.f, 0.f, 0.f};

	float _start_yaw{0.f};
	float _landing_yaw{0.f};
	bool _manual_hold_initialized{false};

	CommandedMode _commanded_mode{CommandedMode::Hover};

	static constexpr float TAKEOFF_HEIGHT_M = 1.0f;
	static constexpr float TAKEOFF_DURATION_S = 2.0f;
	static constexpr float LANDING_DURATION_S = 4.0f;
	static constexpr float LANDED_TARGET_BIAS_M = 0.25f;
	static constexpr float MANUAL_COMMAND_EPS = 0.005f;
	static constexpr float MANUAL_HEIGHT_DEADZONE = 0.10f;
	static constexpr float MANUAL_MAX_CLIMB_RATE_M_S = 0.3f;
	static constexpr float MANUAL_MIN_HEIGHT_M = 0.5f;
	static constexpr float MANUAL_MAX_HEIGHT_M = 2.0f;

	Input _last_input{};
	Output _last_output{};
};
