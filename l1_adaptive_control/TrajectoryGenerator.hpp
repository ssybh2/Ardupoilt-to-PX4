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
Circle = 3
};

enum class CommandedMode : uint8_t {
Hover = 0,
Circle = 1
};

enum class CircleYawMode : uint8_t {
Fixed = 0
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
float circle_radius_m() const { return CIRCLE_RADIUS_M; }
float circle_speed_m_s() const { return CIRCLE_SPEED_M_S; }
float circle_period_s() const;

const Input &last_input() const { return _last_input; }
const Output &last_output() const { return _last_output; }

private:
void set_zero_derivatives(Output &output);
void set_hold_position(Output &output, const float position_ned[3]);
void update_manual_hold_target(const Input &input, Output &output);
float update_manual_height_reference(const Input &input);
void update_hover_target(const Input &input, Output &output);
void update_circle_target(const Input &input, Output &output);
void reset_circle_state();
void sync_hover_reference_from_output(const Output &output);

bool _initialized{false};

hrt_abstime _start_time_us{0};

float _start_position_ned[3]{0.f, 0.f, 0.f};
float _takeoff_target_position_ned[3]{0.f, 0.f, -1.f};
float _hover_position_ned[3]{0.f, 0.f, -1.f};
float _circle_center_position_ned[3]{0.f, 0.f, -1.f};
float _manual_hold_position_ned[3]{0.f, 0.f, 0.f};

float _start_yaw{0.f};
bool _manual_hold_initialized{false};
hrt_abstime _last_update_us{0};

CommandedMode _commanded_mode{CommandedMode::Hover};
CircleYawMode _circle_yaw_mode{CircleYawMode::Fixed};
bool _circle_initialized{false};
float _circle_current_speed_rad_s{0.f};
float _circle_time_offset_s{0.f};
float _circle_current_loop_time_s{0.f};
bool _circle_acc_complete{false};

static constexpr float TAKEOFF_HEIGHT_M = 1.0f;
static constexpr float TAKEOFF_DURATION_S = 2.0f;
static constexpr float CIRCLE_RADIUS_M = 1.0f;
static constexpr float CIRCLE_SPEED_M_S = 0.5f;
static constexpr float MANUAL_HEIGHT_DEADZONE = 0.10f;
static constexpr float MANUAL_MAX_CLIMB_RATE_M_S = 0.3f;
static constexpr float MANUAL_MIN_HEIGHT_M = 0.5f;
static constexpr float MANUAL_MAX_HEIGHT_M = 2.0f;

Input _last_input{};
Output _last_output{};
};
