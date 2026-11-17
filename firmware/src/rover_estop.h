#ifndef ROVER_ESTOP_H
#define ROVER_ESTOP_H

#include <stdint.h>
#include "rover_config.h"

namespace rover {
    class EstopInput{
        public:
        
            EstopInput();

            void update(bool raw_active, uint32-t now_ms)

            bool active() const {return stable_active;}

            bool seeded() const {return seeded_;}

            uint32_t assertions() const {return assertions_;}

        private:

            bool seeded_;
            bool stable_active_;
            bool candidate_;
            uint32_t candidate_since_ms_;
            uint32_t assertions_;
    };
}

#endif