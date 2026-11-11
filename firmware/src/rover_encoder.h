#ifndef ROVER_ENCODER_H
#define ROVER_ENCODER_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {
static const int32_t  ENCODER_TICKS_PER_REV = 2048;
static const int32_t  ENCODER_MAX_RPM       = 120;
static const uint32_t ENCODER_STALE_MS      = 100;
static const uint32_t ENCODER_MAX_STEP_MS   = 100;

struct EncoderFeedback {
    int32_t  count;
    int16_t  rpm;
    uint32_t last_update_ms;
    bool     valid;
};

class SimulatedEncoder {
public:
    SimulatedEncoder();

    void update(int16_t speed_scaled, uint32_t now_ms);

    EncoderFeedback feedback(uint32_t now_ms) const;

private:
    bool     has_update_;
    int32_t  count_;
    int16_t  rpm_;
    uint32_t last_update_ms_;
    int64_t  tick_remainder_;
};
}

#endif
