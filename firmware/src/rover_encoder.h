// rover_encoder.h -- wheel encoder feedback, simulated until real encoders exist.
 
#ifndef ROVER_ENCODER_H
#define ROVER_ENCODER_H
 
#include <stdint.h>
 
#include "rover_protocol.h"
 
namespace rover {
 
// PLACEHOLDERS
static const int32_t  ENCODER_TICKS_PER_REV = 2048;  // ticks per wheel turn
static const int32_t  ENCODER_MAX_RPM       = 120;   // wheel RPM at CMD_MAX
static const uint32_t ENCODER_STALE_MS      = 100;   // older than this = not current
 

struct EncoderFeedback {
    int32_t  count;           // cumulative tics
    int16_t  rpm;             // wheel speed, positive = forward
    uint32_t last_update_ms;  // millis() at the most recent update()
    bool     valid;           // updated at least once, and recently
};
 
class SimulatedEncoder {
public:
    SimulatedEncoder();

    void update(int16_t effort, uint32_t now_ms);
 
    EncoderFeedback feedback(uint32_t now_ms) const;
 
private:
    bool     has_update_;
    int32_t  count_;
    int16_t  rpm_;
    uint32_t last_update_ms_;
    int64_t  tick_remainder_;  // fractions of a tick carried between updates
};
 
}  // namespace rover
 
#endif  // ROVER_ENCODER_H