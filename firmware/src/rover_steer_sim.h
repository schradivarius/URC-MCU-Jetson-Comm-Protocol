// rover_steer_sim.h -- simulated steering feedback that lags the command.
//
// PURE C++. No Arduino.h, no hardware, the clock is passed in. Same shape as
// rover_drive_sim.* and rover_encoder.*, and for the same reason: Task 5 asks
// for the simulated hardware to sit AFTER the steering module so a real
// actuator can replace it without redesigning the firmware. A ramp written
// inline in rover_firmware.cpp cannot be replaced that way, and cannot be unit
// tested either.
//
// WHAT THIS REPLACES
//   The same four lines were copy-pasted into firmware/rover_firmware.cpp and
//   tools/rover_sim.cpp: `steer_fb += 25` per loop iteration, clamped at the
//   target. Two copies free to drift apart, neither tested, and tied to the
//   loop rate rather than to time -- at the simulator's ~2 ms poll that is
//   12500 units/s, so a 300-unit turn completed inside a single 50 ms
//   telemetry period and the Jetson never saw an intermediate value. The task
//   asks for feedback that visibly moves THROUGH the range (0, 5, 9, 13, 15
//   degrees in its example), which that could not show.
//
// WHY A SLEW RATE AND NOT THE EXPONENTIAL APPROACH DriveSim USES
//   A steering actuator is rate limited: it travels at roughly constant speed
//   and then stops at the stop. A drivetrain is inertia limited and approaches
//   its target asymptotically. Modelling both the same way would be less
//   faithful, not more consistent.

#ifndef ROVER_STEER_SIM_H
#define ROVER_STEER_SIM_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {

// PLACEHOLDERS. Exact steering limits are not known yet (the task says so
// explicitly); these are simulation values, labelled as such.
static const int32_t  STEER_SIM_RATE_PER_S  = 2000;  // units/s, full scale is CMD_MAX
static const uint32_t STEER_SIM_MAX_STEP_MS = 1000;  // longest gap simulated in one update
static const uint32_t STEER_STALE_MS        = 100;   // older than this = not current

struct SteerFeedback {
    int16_t  commanded;       // what the steering module asked for
    int16_t  measured;        // where the simulated actuator actually is
    int16_t  error;           // commanded - measured, the gap a controller closes
    uint32_t last_update_ms;  // clock value at the most recent update()
    bool     valid;           // updated at least once, and recently
};

class SteerSim {
public:
    SteerSim();

    // steer_effort is the STEERING MODULE's output, not the raw command, so
    // the mix and the clamp are already applied -- the same place the drive
    // plant is fed from.
    void update(int16_t steer_effort, uint32_t now_ms);

    SteerFeedback feedback(uint32_t now_ms) const;

    int16_t  measured() const     { return measured_; }
    uint32_t lastUpdateMs() const { return last_update_ms_; }

private:
    bool     has_update_;
    int16_t  commanded_;
    int16_t  measured_;
    uint32_t last_update_ms_;
    int32_t  remainder_;  // thousandths of a unit carried between updates, so
                          // a slow rate still moves instead of truncating to 0
};

}  // namespace rover

#endif  // ROVER_STEER_SIM_H
