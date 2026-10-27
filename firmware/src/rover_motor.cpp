#include "rover_motor.h"

namespace rover {

uint32_t pwmFullScale(uint8_t resolution_bits) {
    if (resolution_bits == 0) return 0;
    if (resolution_bits > 16) resolution_bits = 16;
    return (1u << resolution_bits) - 1u;
}

uint32_t servoPulseDuty(uint16_t pulse_us, uint8_t resolution_bits,
                        uint32_t frequency_hz) {
    if (frequency_hz == 0) return 0;
    const uint32_t full_scale = pwmFullScale(resolution_bits);
    const uint32_t period_us = 1000000u / frequency_hz;
    if (period_us == 0) return 0;
    uint64_t pulse = pulse_us;
    if (pulse > period_us) pulse = period_us;
    return static_cast<uint32_t>((pulse * full_scale) / period_us);
}

MotorDuty hbridgeDuty(int16_t effort, uint8_t resolution_bits) {
    MotorDuty result;
    if (effort > CMD_MAX) effort = CMD_MAX;
    if (effort < CMD_MIN) effort = CMD_MIN;
    result.reverse = effort < 0;

    int32_t magnitude = effort < 0 ? -static_cast<int32_t>(effort)
                                   : static_cast<int32_t>(effort);
    if (magnitude <= MOTOR_DEADBAND_EFFORT) {
        result.duty = 0;
        return result;
    }
    result.duty = static_cast<uint32_t>(
        (magnitude * static_cast<int32_t>(pwmFullScale(resolution_bits))) / CMD_MAX);
    return result;
}

}  // namespace rover
