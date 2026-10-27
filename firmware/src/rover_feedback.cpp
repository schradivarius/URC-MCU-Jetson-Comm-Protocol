#include "rover_feedback.h"

namespace rover {

FeedbackAge::FeedbackAge(MillisFn now_ms, uint32_t stale_ms)
    : now_ms_(now_ms), stale_ms_(stale_ms) {}

void FeedbackAge::noteSample() {
    last_sample_ms_ = now_ms_();
    have_sample_ = true;
}

bool FeedbackAge::hasSample() const {return have_sample_;}
bool FeedbackAge::isStale() const {
    if(!have_sample_){
        return true;
    }
    return elapsed() >= stale_ms_;
}

uint16_t FeedbackAge::ageMs() const {
    if(!have_sample_){
        return CMD_AGE_UNKNOWN;
    }
    // clampCmdAgeMs is named for command age but clamps any age in ms.
    return clampCmdAgeMs(static_cast<int64_t>(elapsed()));
}

uint32_t FeedbackAge::elapsed() const {
    return now_ms_() - last_sample_ms_;
}


} // namespace rover