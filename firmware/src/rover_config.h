#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <stdint.h>

namespace rover {
static const uint8_t PROTOCOL_VERSION = 8;

static const uint32_t SERIAL_BAUD_HZ = 115200;

static const uint32_t DEFAULT_WATCHDOG_TIMEOUT_MS = 300;
static const uint32_t DEFAULT_TELEMETRY_PERIOD_MS = 50;
static const uint32_t HW_WATCHDOG_MS              = 1000;

static const uint32_t LINK_DEGRADED_HOLD_MS = 1000;

enum : uint32_t {
    CAN_ID_CONTROL        = 0x100,
    CAN_ID_TELEM_DRIVE_L  = 0x200,
    CAN_ID_TELEM_DRIVE_R  = 0x201,
    CAN_ID_TELEM_POWER    = 0x202,
    CAN_ID_TELEM_STATE    = 0x203,
    CAN_ID_TELEM_CMD_ECHO = 0x204,
    CAN_ID_TELEM_FEEDBACK = 0x205,
};

enum : uint8_t {
    FRAME_DLC        = 8,
    FRAME_PAYLOAD    = 6,
    FRAME_SEQ_OFFSET = 6,
    FRAME_CRC_OFFSET = 7,
};

enum : uint8_t {
    MODE_SAFE       = 0,
    MODE_TELEOP     = 1,
    MODE_AUTONOMOUS = 2,
    MODE_FAULT      = 3,
};
static const uint8_t MODE_MAX = MODE_AUTONOMOUS;

static const int16_t OVER_CURRENT_CA = 4000;
static const int16_t UNDERVOLTAGE_CV = 2000;

static const uint32_t FEEDBACK_STALE_MS = 150;

static const uint16_t PWM_NEUTRAL_US = 1500;
static const uint16_t PWM_SPAN_US    = 500;

static const int16_t CMD_MIN = -1000;
static const int16_t CMD_MAX =  1000;
}

#endif
