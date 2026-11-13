#ifndef ROVER_DRIVE_H
#define ROVER_DRIVE_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {
struct DriveOutputs {
    int16_t  left_effort;
    int16_t  right_effort;
    int16_t  steer_effort;
    uint16_t left_pwm_us;
    uint16_t right_pwm_us;
    uint16_t steer_pwm_us;
    bool     energized;
};

class DriveModule {
public:
    DriveModule();

    const DriveOutputs& update(int16_t drive, int16_t steer, bool energized);

    const DriveOutputs& outputs() const { return out_; }

    uint8_t health() const;

    uint32_t rangeViolations() const { return range_violations_; }

    uint32_t updates() const { return updates_; }

private:
    void neutral();

    DriveOutputs out_;
    bool     started_;
    bool     range_fault_;
    bool     mix_saturated_;
    uint32_t range_violations_;
    uint32_t updates_;
};

uint16_t effortToPwmUs(int16_t effort);
}

#endif
