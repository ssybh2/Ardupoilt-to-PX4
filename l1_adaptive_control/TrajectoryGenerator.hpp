#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

class TrajectoryGenerator
{
public:
enum class Mode : uint8_t {
WaitForValidState = 0,
Takeoff = 1,
Hold = 2
};

struct Input {
hrt_abstime timestamp_us{0};

float current_position_ned[3]{0.f, 0.f, 0.f};
float current_yaw{0.f};

bool state_valid_for_control{false};
bool armed{false};
bool failsafe{false};
uint8_t nav_state{0};
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

const Input &last_input() const { return _last_input; }
const Output &last_output() const { return _last_output; }

private:
void set_zero_derivatives(Output &output);

bool _initialized{false};

hrt_abstime _start_time_us{0};

float _start_position_ned[3]{0.f, 0.f, 0.f};
float _takeoff_target_position_ned[3]{0.f, 0.f, -1.f};

float _start_yaw{0.f};

static constexpr float TAKEOFF_HEIGHT_M = 1.0f;
static constexpr float TAKEOFF_DURATION_S = 2.0f;

Input _last_input{};
Output _last_output{};
};
