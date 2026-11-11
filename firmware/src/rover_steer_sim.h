#ifndef ROVER_STEER_SIM_H
#define ROVER_STEER_SIM_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {
static const int32_t  STEER_SIM_RATE_PER_S  = 2000;
static const uint32_t STEER_SIM_MAX_STEP_MS = 1000;
static const uint32_t STEER_STALE_MS        = 100;

struct SteerFeedback {
    int16_t  commanded;
    int16_t  measured;
    int16_t  error;
    uint32_t last_update_ms;
    bool     valid;
};

class SteerSim {
public:
    SteerSim();

    void update(int16_t steer_effort, uint32_t now_ms);

    SteerFeedback feedback(uint32_t now_ms) const;

    int16_t  measured() const     { return measured_; }
    uint32_t lastUpdateMs() const { return last_update_ms_; }

private:
    bool     has_update_;
    int16_t  commanded_;
    int16_t  measured_;
    uint32_t last_update_ms_;
    int32_t  remainder_;
};
}

#endif
