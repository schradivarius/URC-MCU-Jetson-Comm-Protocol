#include "rover_drive.h"

namespace rover {
static_assert(CMD_MAX > 0, "CMD_MAX must be positive; it divides below");
static_assert(CMD_MIN == -CMD_MAX, "the command range is assumed symmetric");

namespace {
int16_t clampEffort(int32_t value) {
    if (value > CMD_MAX) return CMD_MAX;
    if (value < CMD_MIN) return CMD_MIN;
    return static_cast<int16_t>(value);
}
}

uint16_t effortToPwmUs(int16_t effort) {
    const int32_t clamped = clampEffort(effort);
    const int32_t offset =
        (clamped * static_cast<int32_t>(PWM_SPAN_US)) / static_cast<int32_t>(CMD_MAX);
    return static_cast<uint16_t>(static_cast<int32_t>(PWM_NEUTRAL_US) + offset);
}

DriveModule::DriveModule()
    : started_(false),
      range_fault_(false),
      mix_saturated_(false),
      range_violations_(0),
      updates_(0) {
    neutral();
}

void DriveModule::neutral() {
    out_.left_effort   = 0;
    out_.right_effort  = 0;
    out_.steer_effort  = 0;
    out_.left_pwm_us   = PWM_NEUTRAL_US;
    out_.right_pwm_us  = PWM_NEUTRAL_US;
    out_.steer_pwm_us  = PWM_NEUTRAL_US;
    out_.energized     = false;
}

const DriveOutputs& DriveModule::update(int16_t drive, int16_t steer, bool energized) {
    started_ = true;
    updates_++;
    mix_saturated_ = false;

    if (!isValidCommand(drive) || !isValidCommand(steer)) {
        range_fault_ = true;
        range_violations_++;
    }
    const int32_t d = clampEffort(drive);
    const int32_t s = clampEffort(steer);

    if (!energized) {
        neutral();
        return out_;
    }

    int32_t left  = d + s;
    int32_t right = d - s;
    int32_t peak  = left >= 0 ? left : -left;
    const int32_t right_mag = right >= 0 ? right : -right;
    if (right_mag > peak) peak = right_mag;
    if (peak > CMD_MAX) {
        left  = (left  * CMD_MAX) / peak;
        right = (right * CMD_MAX) / peak;
        mix_saturated_ = true;
    }

    out_.left_effort  = clampEffort(left);
    out_.right_effort = clampEffort(right);
    out_.steer_effort = static_cast<int16_t>(s);
    out_.left_pwm_us  = effortToPwmUs(out_.left_effort);
    out_.right_pwm_us = effortToPwmUs(out_.right_effort);
    out_.steer_pwm_us = effortToPwmUs(out_.steer_effort);
    out_.energized    = true;
    return out_;
}

uint8_t DriveModule::health() const {
    if (!started_)      return CTRL_HEALTH_NOT_REPORTED;
    if (range_fault_)   return CTRL_HEALTH_FAULT;
    if (mix_saturated_) return CTRL_HEALTH_DEGRADED;
    return CTRL_HEALTH_OK;
}
}
