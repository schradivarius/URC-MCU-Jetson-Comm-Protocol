#include "rover_drive_sim.h"

#include "rover_protocol.h"

namespace rover {

DriveSim::DriveSim(uint32_t seed)
    : rng_(seed ? seed : 1),   // xorshift never leaves 0
      started_(false),
      last_ms_(0),
      last_dt_ms_(0),
      frac_left_(0.0f),
      frac_right_(0.0f) {
    left_  = {0.0f, 0.0f, 0, 0.0f};
    right_ = {0.0f, 0.0f, 0, 0.0f};
}

// xorshift32: same sequence on the Teensy and a laptop, unlike rand().
float DriveSim::random01() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return static_cast<float>(rng_ >> 8) / 16777216.0f;   // [0, 1)
}

void DriveSim::update(int16_t left_effort, int16_t right_effort, bool energized,
                      uint32_t now_ms) {
    if (!started_) {
        started_ = true;
        last_ms_ = now_ms;
        return;
    }
    const uint32_t dt = now_ms - last_ms_;   // unsigned: correct across millis() wrap
    if (dt == 0) return;
    last_ms_ = now_ms;
    last_dt_ms_ = dt;

    // One random factor for both wheels, so equal commands stay equal.
    const float jitter = 1.0f + (2.0f * random01() - 1.0f) * SIM_STEP_JITTER;
    stepWheel(left_,  frac_left_,  left_effort,  energized, static_cast<float>(dt), jitter);
    stepWheel(right_, frac_right_, right_effort, energized, static_cast<float>(dt), jitter);
}

void DriveSim::stepWheel(WheelFeedback& w, float& frac, int16_t effort,
                         bool energized, float dt_ms, float jitter) {
    w.commanded_mps = energized
        ? static_cast<float>(effort) / static_cast<float>(CMD_MAX) * SIM_MAX_SPEED_MPS
        : 0.0f;

    float k = dt_ms / (energized ? SIM_RESPONSE_MS : SIM_BRAKE_MS) * jitter;
    if (k > 1.0f) k = 1.0f;
    w.measured_mps += (w.commanded_mps - w.measured_mps) * k;

    const float mag = w.measured_mps >= 0.0f ? w.measured_mps : -w.measured_mps;
    if (w.commanded_mps == 0.0f && mag < SIM_STOPPED_MPS) w.measured_mps = 0.0f;

    frac += w.measured_mps * (dt_ms / 1000.0f) * SIM_TICKS_PER_M;
    const int32_t whole = static_cast<int32_t>(frac);
    frac -= static_cast<float>(whole);
    w.encoder_ticks = wrapI32(static_cast<int64_t>(w.encoder_ticks) + whole);
    w.ticks_per_s = w.measured_mps * SIM_TICKS_PER_M;
}

}  // namespace rover
