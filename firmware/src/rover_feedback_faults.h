// rover_feedback_faults.h -- controlled failures of the SIMULATED feedback.
//
// PURE C++. No Arduino.h, no hardware, the clock is passed in.
//
// Task 5 asks for "a few controlled failure conditions" so the detection
// added in v0.8 can be shown to work, and names four:
//
//   1. encoder feedback stops updating
//   2. steering feedback stops updating
//   3. encoder or steering feedback contains an impossible/invalid value
//   4. feedback is intentionally delayed
//
// WHERE THIS SITS, AND WHY IT MATTERS
//   Between the simulated plant and the controller, on the SIMULATED side of
//   the boundary. It is a fake broken sensor, not firmware logic: it corrupts
//   readings exactly the way failing hardware would, and the controller is
//   given no idea that anything is unusual. Putting the injection inside
//   RoverController instead would be a test that proves the detector agrees
//   with itself.
//
//   It therefore disappears with the rest of the simulation when real sensors
//   arrive -- the same way DriveSim, SimulatedEncoder and SteerSim do.
//
// WHY A DELAY LINE AND NOT A SHIFTED TIMESTAMP
//   "Delayed" and "frozen" are different faults and must not collapse into
//   one. A frozen channel repeats one reading forever; a delayed channel
//   reports CORRECT readings LATE, so the values still track the command and
//   only their age gives the problem away. Faking the delay by subtracting
//   from the timestamp would produce a frozen channel wearing a delayed
//   channel's age, and a detector tuned against it would be tuned against a
//   fault that cannot happen. So this keeps a real ring of past samples and
//   serves the newest one old enough to be due.

#ifndef ROVER_FEEDBACK_FAULTS_H
#define ROVER_FEEDBACK_FAULTS_H

#include <stdint.h>

#include "rover_encoder.h"
#include "rover_steer_sim.h"

namespace rover {

// How far behind a delayed channel runs. Longer than FEEDBACK_STALE_MS, so
// the delay is something the controller notices rather than absorbs.
static const uint32_t FB_FAULT_DELAY_MS = 250;

// The delay line, sized in TIME rather than in calls. apply() runs once per
// loop and the loop rate is not fixed -- the Teensy's is sub-millisecond, the
// host simulator's a couple of milliseconds, the unit tests' ten -- so a ring
// of N calls holds a different amount of history on each, and on the fast ones
// not enough: a 64-call ring at the simulator's rate spans about 128 ms, which
// cannot produce a 250 ms delay at all. It would report a 128 ms one and look
// like a working fault.
//
// So a sample is taken at most every FB_FAULT_SAMPLE_MS, and the ring spans
// (FB_FAULT_HISTORY - 1) * FB_FAULT_SAMPLE_MS no matter who is calling. The
// static_assert below is what keeps that span longer than the delay being
// asked for, so changing either constant cannot quietly break it.
//
// The cost is that the delayed channel is quantised to FB_FAULT_SAMPLE_MS,
// which is 5 ms against a 50 ms telemetry period: invisible.
static const uint32_t FB_FAULT_SAMPLE_MS = 5;
static const uint8_t  FB_FAULT_HISTORY   = 64;

enum : uint8_t {
    FB_FAULT_NONE               = 0,
    FB_FAULT_ENCODER_FROZEN     = 1,  // encoder feedback stops updating
    FB_FAULT_STEER_FROZEN       = 2,  // steering feedback stops updating
    FB_FAULT_ENCODER_IMPOSSIBLE = 3,  // an encoder value the hardware cannot give
    FB_FAULT_STEER_IMPOSSIBLE   = 4,  // a steering value the actuator cannot reach
    FB_FAULT_DELAYED            = 5,  // both channels run FB_FAULT_DELAY_MS late
    FB_FAULT_MAX                = 5,
};

// A ring that cannot reach FB_FAULT_DELAY_MS would report a shorter delay
// than it was asked for, which is a silently wrong test rather than a failing
// one.
static_assert((FB_FAULT_HISTORY - 1) * FB_FAULT_SAMPLE_MS >= FB_FAULT_DELAY_MS,
              "the delay line must span at least FB_FAULT_DELAY_MS");

const char* feedbackFaultName(uint8_t fault);

// Parses a name as accepted on the command line ("encoder-frozen",
// "steer-frozen", "encoder-impossible", "steer-impossible", "delayed",
// "none"). Returns false and leaves `out` alone if the name is unknown --
// a typo must not silently run a healthy plant and pass.
bool parseFeedbackFault(const char* name, uint8_t& out);

class FeedbackFaultInjector {
public:
    FeedbackFaultInjector();

    // Arms a fault, or FB_FAULT_NONE to clear it. Re-arming the same fault is
    // a no-op, so a caller may set it every loop without restarting it.
    void arm(uint8_t fault);
    uint8_t armed() const { return fault_; }

    // Takes the plant's honest readings and returns what a failing sensor
    // would have reported instead. Call once per loop, before handing the
    // readings to the controller.
    void apply(EncoderFeedback& encoder, SteerFeedback& steer, uint32_t now_ms);

private:
    struct Sample {
        uint32_t taken_ms;
        EncoderFeedback encoder;
        SteerFeedback   steer;
    };

    // Records at most one sample per FB_FAULT_SAMPLE_MS; returns nothing
    // because a skipped sample is normal, not a failure.
    void record(const EncoderFeedback& e, const SteerFeedback& s, uint32_t now_ms);
    const Sample* due(uint32_t now_ms) const;

    uint8_t  fault_;
    bool     have_frozen_;
    EncoderFeedback frozen_encoder_;
    SteerFeedback   frozen_steer_;

    Sample  history_[FB_FAULT_HISTORY];
    uint8_t count_;   // samples recorded, capped at FB_FAULT_HISTORY
    uint8_t next_;    // where the next sample goes
};

}  // namespace rover

#endif  // ROVER_FEEDBACK_FAULTS_H
