#ifndef ROVER_MOTOR_H
#define ROVER_MOTOR_H

#include <stdint.h>

#include "rover_config.h"

namespace rover {

struct MotorDuty {
    uint32_t duty;
    bool     reverse;
};

uint32_t pwmFullScale(uint8_t resolution_bits);

uint32_t servoPulseDuty(uint16_t pulse_us, uint8_t resolution_bits,
                        uint32_t frequency_hz);

MotorDuty hbridgeDuty(int16_t effort, uint8_t resolution_bits);

}  // namespace rover

#endif  // ROVER_MOTOR_H
