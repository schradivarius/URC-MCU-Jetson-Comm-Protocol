// rover_drive_sim.h -- simulated drive feedback.
//
// Measured wheel velocity eases toward the commanded velocity instead of
// matching it instantly, so commanded and measured state can be told apart.
// Not a physics model. Pure C++, so it builds for the Teensy and the tests.

#ifndef ROVER_DRIVE_SIM_H
#define ROVER_DRIVE_SIM_H

#include <stdint.h>

namespace rover {

static const float SIM_MAX_SPEED_MPS  = 1.0f;    // effort CMD_MAX maps to this
static const float SIM_TICKS_PER_M    = 1000.0f;
static const float SIM_RESPONSE_MS    = 100.0f;  // time constant while driving
static const float SIM_BRAKE_MS       = 30.0f;   // time constant when de-energized
static const float SIM_STEP_JITTER    = 0.3f;    // random +/-30% on each step
static const float SIM_STOPPED_MPS    = 0.005f;  // below this with no command, snap to 0

struct WheelFeedback {
    float   commanded_mps;
    float   measured_mps;
    int32_t encoder_ticks;   // wraps
    float   ticks_per_s;
};

class DriveSim {
public:
    explicit DriveSim(uint32_t seed = 1);

    void update(int16_t left_effort, int16_t right_effort, bool energized,
                uint32_t now_ms);

    const WheelFeedback& left() const  { return left_; }
    const WheelFeedback& right() const { return right_; }
    uint32_t lastUpdateMs() const { return last_ms_; }
    uint32_t lastDtMs() const     { return last_dt_ms_; }

private:
    float random01();
    void stepWheel(WheelFeedback& w, float& frac, int16_t effort, bool energized,
                   float dt_ms, float jitter);

    uint32_t rng_;
    bool     started_;
    uint32_t last_ms_;
    uint32_t last_dt_ms_;
    WheelFeedback left_, right_;
    float frac_left_, frac_right_;   // sub-tick distance carried between updates
};

}  // namespace rover

#endif  // ROVER_DRIVE_SIM_H
