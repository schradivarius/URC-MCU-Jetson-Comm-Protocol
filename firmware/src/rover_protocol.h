#ifndef ROVER_PROTOCOL_H
#define ROVER_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#include "rover_config.h"

namespace rover {
enum : uint8_t {
    CTRL_FLAG_STOP           = 0x01,
    CTRL_FLAG_AUTONOMY_ABORT = 0x02,
    CTRL_FLAG_RETURN_REQUEST = 0x04,
    CTRL_INDICATOR_MASK      = 0x38,
    CTRL_INDICATOR_SHIFT     = 3,
    CTRL_FLAG_C2_LOST        = 0x40,
    CTRL_FLAG_FROM_AUTONOMY  = 0x80,
};

bool sourceOwnsMode(bool from_autonomy, uint8_t mode);
bool transitionAllowed(uint8_t from, uint8_t to, bool from_autonomy);

enum : uint8_t {
    INDICATOR_OFF        = 0,
    INDICATOR_TELEOP     = 1,
    INDICATOR_AUTONOMOUS = 2,
    INDICATOR_ARRIVED    = 3,
    INDICATOR_FAULT      = 4,
    INDICATOR_MAX        = 7,
};

enum : uint8_t {
    CMD_SOURCE_NONE     = 0,
    CMD_SOURCE_OPERATOR = 1,
    CMD_SOURCE_AUTONOMY = 2,
    CMD_SOURCE_MAX      = 3,
};

const char* cmdSourceName(uint8_t source);

enum : uint16_t {
    FAULT_COMM_TIMEOUT   = 0x0001,
    FAULT_OVER_CURRENT   = 0x0002,
    FAULT_ESTOP_ACTIVE   = 0x0004,
    FAULT_ENCODER_FAULT  = 0x0008,
    FAULT_UNDERVOLTAGE   = 0x0010,
    FAULT_FIRMWARE_FAULT = 0x0020,
    FAULT_PROTOCOL_ERROR = 0x0040,
    FAULT_C2_LINK_LOST   = 0x0080,
    FAULT_SEQ_GAP        = 0x0100,
    FAULT_CRC_ERROR      = 0x0200,
    FAULT_CMD_SOURCE     = 0x0400,
    FAULT_STALE_FEEDBACK = 0x0800,
    FAULT_STEERING_FAULT = 0x1000,
};
static const uint16_t FAULT_CRITICAL_MASK = FAULT_OVER_CURRENT | FAULT_ESTOP_ACTIVE;
enum : uint8_t {
    LINK_OK           = 0,
    LINK_DEGRADED     = 1,
    LINK_LOST         = 2,
    LINK_NOT_REPORTED = 3,
};

enum : uint8_t {
    CTRL_HEALTH_OK           = 0,
    CTRL_HEALTH_DEGRADED     = 1,
    CTRL_HEALTH_FAULT        = 2,
    CTRL_HEALTH_NOT_REPORTED = 3,
};

enum : uint8_t {
    CMD_STATUS_APPLIED      = 0,
    CMD_STATUS_STOPPED      = 1,
    CMD_STATUS_WRONG_SOURCE = 2,
    CMD_STATUS_NO_COMMAND   = 3,
    CMD_STATUS_MAX          = 7,
    CMD_STATUS_MODE_REFUSED = 4,
};

enum : uint16_t {
    CMD_AGE_UNKNOWN = 0xFFFF,
    CMD_AGE_MAX     = 0xFFFE,
};

enum : uint16_t {
    FEEDBACK_AGE_UNKNOWN = 0xFFFF,
    FEEDBACK_AGE_MAX     = 0xFFFE,
};

enum : uint8_t {
    FB_FLAG_ENCODER_STALE   = 0x01,
    FB_FLAG_STEER_STALE     = 0x02,
    FB_FLAG_ENCODER_INVALID = 0x04,
    FB_FLAG_STEER_INVALID   = 0x08,
    FB_FLAGS_MASK           = 0x0F,
    FB_FLAGS_RESERVED_MASK  = 0xF0,
};

static const int32_t INT32_MIN_V = -2147483647 - 1;
static const int32_t INT32_MAX_V = 2147483647;

bool isCommandableMode(uint8_t mode);
bool modePermitsMotion(uint8_t mode);
bool isKnownIndicator(uint8_t indicator);

bool isValidCommand(int16_t value);

uint8_t crc8Update(uint8_t crc, const uint8_t* data, size_t len);

uint8_t crc8(const uint8_t* data, size_t len);

uint8_t frameCrc8(uint32_t can_id, const uint8_t* payload, size_t len);

int32_t wrapI32(int64_t value);

uint16_t clampCmdAgeMs(int64_t age_ms);

uint16_t clampFeedbackAgeMs(int64_t age_ms);

size_t faultNames(uint16_t bitmask, const char** out, size_t cap);

uint8_t seqDelta(uint8_t previous, uint8_t current);

struct ControlMsg {
    int16_t drive_cmd;
    int16_t steer_cmd;
    uint8_t mode;
    bool    stop;
    bool    autonomy_abort;
    bool    return_request;
    uint8_t indicator_request;
    bool    c2_lost;
    bool    from_autonomy;
    uint8_t seq;
};

struct TelemetryDriveL {
    int32_t enc_left;
    int16_t steer_fb;
};

struct TelemetryDriveR {
    int32_t enc_right;
    uint16_t cmd_age_ms;
};

struct TelemetryPower {
    int16_t  current_ca;
    int16_t  voltage_cv;
    uint16_t fault_status;
};

struct TelemetryState {
    uint8_t mode;
    uint8_t jetson_link;
    uint8_t c2_link;
    uint8_t controller_health;
    uint8_t indicator_state;
    uint8_t reserved;
};

struct TelemetryCmdEcho {
    int16_t drive_applied;
    int16_t steer_applied;
    uint8_t ack_seq;
    uint8_t status;
};

struct TelemetryFeedback {
    uint16_t encoder_age_ms;
    uint16_t steer_age_ms;
    uint8_t  cmd_source;
    uint8_t  flags;
};

enum DecodeResult : uint8_t {
    DECODE_OK = 0,
    DECODE_BAD_DLC,
    DECODE_BAD_CRC,
    DECODE_BAD_MODE,
    DECODE_BAD_FLAGS,
    DECODE_BAD_INDICATOR,
    DECODE_BAD_RANGE,
};

const char* decodeResultName(DecodeResult r);

enum : uint8_t {
    CMD_ECHO_STATUS_MASK   = 0x07,
    CMD_ECHO_DECODE_MASK   = 0x38,
    CMD_ECHO_DECODE_SHIFT  = 3,
    CMD_ECHO_RETURN_ACTIVE = 0x40,
    CMD_ECHO_RESERVED_MASK = 0x80,
};

uint8_t      packCmdStatus(uint8_t status, DecodeResult decode, bool return_active);
uint8_t      cmdStatusOf(uint8_t packed);
DecodeResult decodeResultOf(uint8_t packed);
bool         returnActiveOf(uint8_t packed);
const char*  cmdStatusName(uint8_t status);

uint8_t      encodeControl(const ControlMsg& msg, uint8_t* buf);
DecodeResult decodeControl(uint32_t can_id, const uint8_t* buf, uint8_t len,
                           ControlMsg& out);

uint8_t      encodeTelemetryDriveL(const TelemetryDriveL& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryDriveL(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                   TelemetryDriveL& out, uint8_t& seq_out);

uint8_t      encodeTelemetryDriveR(const TelemetryDriveR& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryDriveR(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                   TelemetryDriveR& out, uint8_t& seq_out);

uint8_t      encodeTelemetryPower(const TelemetryPower& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryPower(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                  TelemetryPower& out, uint8_t& seq_out);

uint8_t      encodeTelemetryState(const TelemetryState& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryState(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                  TelemetryState& out, uint8_t& seq_out);

uint8_t      encodeTelemetryCmdEcho(const TelemetryCmdEcho& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryCmdEcho(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                    TelemetryCmdEcho& out, uint8_t& seq_out);

uint8_t      encodeTelemetryFeedback(const TelemetryFeedback& msg, uint8_t seq, uint8_t* buf);
DecodeResult decodeTelemetryFeedback(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                     TelemetryFeedback& out, uint8_t& seq_out);
}

#endif
