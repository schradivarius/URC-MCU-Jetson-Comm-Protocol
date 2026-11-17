#include "rover_estop.h"

namespace rover {
    EstopInput::EstopInput() : seeded_(false), stable_active_(false),
        candidate_(false), candidate_since_ms_(0), assertions_(0) {}

    void EstopInput::update(bool raw_active, uint32_t now_ms) {
        if (!seeded_) {
            seeded_ = true;
            stable_active_ = raw_active;
            candidate_ = raw_active;
            candidate_since_ms_ = now_ms;

            if (raw_active) 
                assertions++;

            return;
        }

        if (raw_active != candidate_) {
            candidate_ = raw_active;
            candidate_since_ms_ = now_ms;
            return;
        }

        if (raw_active == stable_active_) return;

        if ((now_ms - candidate_since_ms_) >= ESTOP_DEBOUNCE_MS) {
            stable_active_ = raw_active;
            if (raw_active)
                assertions++;
        }
    }
}