#ifndef ROVER_FEEDBACK_FAULTS_H
#define ROVER_FEEDBACK_FAULTS_H

#include <stdint.h>

#include "rover_encoder.h"
#include "rover_steer_sim.h"

namespace rover {
static const uint32_t FB_FAULT_DELAY_MS = 250;

static const uint32_t FB_FAULT_SAMPLE_MS = 5;
static const uint8_t  FB_FAULT_HISTORY   = 64;

enum : uint8_t {
    FB_FAULT_NONE               = 0,
    FB_FAULT_ENCODER_FROZEN     = 1,
    FB_FAULT_STEER_FROZEN       = 2,
    FB_FAULT_ENCODER_IMPOSSIBLE = 3,
    FB_FAULT_STEER_IMPOSSIBLE   = 4,
    FB_FAULT_DELAYED            = 5,
    FB_FAULT_MAX                = 5,
};

static_assert((FB_FAULT_HISTORY - 1) * FB_FAULT_SAMPLE_MS >= FB_FAULT_DELAY_MS,
              "the delay line must span at least FB_FAULT_DELAY_MS");

const char* feedbackFaultName(uint8_t fault);

bool parseFeedbackFault(const char* name, uint8_t& out);

class FeedbackFaultInjector {
public:
    FeedbackFaultInjector();

    void arm(uint8_t fault);
    uint8_t armed() const { return fault_; }

    void apply(EncoderFeedback& encoder, SteerFeedback& steer, uint32_t now_ms);

private:
    struct Sample {
        uint32_t taken_ms;
        EncoderFeedback encoder;
        SteerFeedback   steer;
    };

    void record(const EncoderFeedback& e, const SteerFeedback& s, uint32_t now_ms);
    const Sample* due(uint32_t now_ms) const;

    uint8_t  fault_;
    bool     have_frozen_;
    EncoderFeedback frozen_encoder_;
    SteerFeedback   frozen_steer_;

    Sample  history_[FB_FAULT_HISTORY];
    uint8_t count_;
    uint8_t next_;
};
}

#endif
