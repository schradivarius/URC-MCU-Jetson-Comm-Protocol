// rover_drive_sim.h -- simulated drive feedback that lags the command.

#ifndef ROVER_DRIVE_SIM_H
#define ROVER_DRIVE_SIM_H

#include <stdint.h>

namespace rover {

static const float SIM_MAX_SPEED_MPS  = 1.0f;    // speed at full effort (CMD_MAX)
static const float SIM_TICKS_PER_M    = 1000.0f;
static const float SIM_RESPONSE_MS    = 100.0f;  // how fast measured follows command
static const float SIM_BRAKE_MS       = 30.0f;   // how fast it slows when stopped
static const float SIM_STEP_JITTER    = 0.3f;    // random +/-30% on each step
static const float SIM_STOPPED_MPS    = 0.005f;  // snap to 0 below this when stopping

struct WheelFeedback {
    float   commanded_mps;
    float   measured_mps;
    int32_t encoder_ticks;
    float   ticks_per_s;
};

class DriveSim {
public:
    // seed picks the random sequence; the same seed gives the same run.
    explicit DriveSim(uint32_t seed = 1);

    // Advances both wheels to now_ms using the drive stage's output.
    void update(int16_t left_effort, int16_t right_effort, bool energized,
                uint32_t now_ms);

    const WheelFeedback& left() const  { return left_; }
    const WheelFeedback& right() const { return right_; }

    // Time of the last update, and the time since the one before it.
    uint32_t lastUpdateMs() const { return last_ms_; }
    uint32_t lastDtMs() const     { return last_dt_ms_; }

private:
    // Next random number in [0, 1).
    float random01();

    // Moves one wheel's measured speed toward its command and adds encoder ticks.
    void stepWheel(WheelFeedback& w, float& frac, int16_t effort, bool energized,
                   float dt_ms, float jitter);

    uint32_t rng_;
    bool     started_;
    uint32_t last_ms_;
    uint32_t last_dt_ms_;
    WheelFeedback left_, right_;
    float frac_left_, frac_right_;   // leftover partial tick
};

}  // namespace rover

#endif  // ROVER_DRIVE_SIM_H
