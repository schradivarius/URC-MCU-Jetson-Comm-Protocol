#include "rover_steer_sim.h"

namespace rover {
SteerSim::SteerSim()
    : has_update_(false),
      commanded_(0),
      measured_(0),
      last_update_ms_(0),
      remainder_(0) {}

void SteerSim::update(int16_t steer_effort, uint32_t now_ms) {
    commanded_ = steer_effort;

    const uint32_t gap_ms = has_update_ ? (now_ms - last_update_ms_) : 0;
    const uint32_t dt_ms = gap_ms < STEER_SIM_MAX_STEP_MS ? gap_ms
                                                          : STEER_SIM_MAX_STEP_MS;

    last_update_ms_ = now_ms;
    has_update_     = true;

    const int32_t gap = static_cast<int32_t>(commanded_) - measured_;
    if (gap == 0) {
        remainder_ = 0;
        return;
    }

    remainder_ += static_cast<int32_t>(STEER_SIM_RATE_PER_S *
                                       static_cast<int64_t>(dt_ms));
    const int32_t step = remainder_ / 1000;
    remainder_ -= step * 1000;
    if (step == 0) return;

    const int32_t travel = (gap > 0) ? (step < gap ? step : gap)
                                     : (-step > gap ? -step : gap);
    measured_ = static_cast<int16_t>(measured_ + travel);
}

SteerFeedback SteerSim::feedback(uint32_t now_ms) const {
    SteerFeedback f;
    f.commanded      = commanded_;
    f.measured       = measured_;
    f.error          = static_cast<int16_t>(commanded_ - measured_);
    f.last_update_ms = last_update_ms_;
    f.valid          = has_update_ && (now_ms - last_update_ms_) <= STEER_STALE_MS;
    return f;
}
}
