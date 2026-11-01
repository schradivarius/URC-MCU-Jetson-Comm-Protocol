#include "rover_encoder.h"

namespace rover {

// One tick, expressed in the units tick_remainder_ counts in.
static const int64_t TICK_UNITS = static_cast<int64_t>(CMD_MAX) * 60000;

SimulatedEncoder::SimulatedEncoder()
    : has_update_(false),
      count_(0),
      rpm_(0),
      last_update_ms_(0),
      tick_remainder_(0) {}

void SimulatedEncoder::update(int16_t effort, uint32_t now_ms) {
    const uint32_t dt_ms = has_update_ ? (now_ms - last_update_ms_) : 0;

    tick_remainder_ += static_cast<int64_t>(effort) * ENCODER_MAX_RPM *
                       ENCODER_TICKS_PER_REV * dt_ms;
    const int64_t whole_ticks = tick_remainder_ / TICK_UNITS;
    tick_remainder_ -= whole_ticks * TICK_UNITS;

    count_ = wrapI32(static_cast<int64_t>(count_) + whole_ticks);
    rpm_ = static_cast<int16_t>(static_cast<int32_t>(effort) * ENCODER_MAX_RPM / CMD_MAX);
    last_update_ms_ = now_ms;
    has_update_ = true;
}

EncoderFeedback SimulatedEncoder::feedback(uint32_t now_ms) const {
    EncoderFeedback f;
    f.count = count_;
    f.rpm = rpm_;
    f.last_update_ms = last_update_ms_;
    f.valid = has_update_ && (now_ms - last_update_ms_) <= ENCODER_STALE_MS;
    return f;
}

} 
