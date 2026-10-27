#ifndef ROVER_FEEDBACK_H
#define ROVER_FEEDBACK_H

#include "rover_protocol.h"

namespace rover {
class FeedbackAge {
    public:
        FeedbackAge(MillisFn now_ms, uint32_t stale_ms);

        void noteSample();
        
        bool hasSample() const;

        bool isStale() const;

        uint16_t ageMs() const;
    private:
        uint32_t elapsed() const;
        MillisFn now_ms_;
        uint32_t stale_ms_;
        uint32_t last_sample_ms_ = 0;
        bool have_sample_ = false;
};
}

#endif  // ROVER_FEEDBACK_H