import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rover_config import (  # noqa: E402,F401
    CAN_ID_CONTROL, CAN_ID_TELEM_DRIVE_L, CAN_ID_TELEM_DRIVE_R,
    CAN_ID_TELEM_POWER, CAN_ID_TELEM_STATE, CAN_ID_TELEM_CMD_ECHO,
    CAN_ID_TELEM_FEEDBACK,
    MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS, MODE_FAULT, MODE_MAX,
    CMD_MIN, CMD_MAX, PROTOCOL_VERSION, PWM_NEUTRAL_US, PWM_SPAN_US,
    FEEDBACK_STALE_MS,
)

TELEM_IDS = (CAN_ID_TELEM_DRIVE_L, CAN_ID_TELEM_DRIVE_R,
             CAN_ID_TELEM_POWER, CAN_ID_TELEM_STATE, CAN_ID_TELEM_CMD_ECHO,
             CAN_ID_TELEM_FEEDBACK)

MODE_NAMES = {MODE_SAFE: "SAFE", MODE_TELEOP: "TELEOP",
              MODE_AUTONOMOUS: "AUTONOMOUS", MODE_FAULT: "FAULT"}

COMMANDABLE_MODES  = (MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS)
MOTION_MODES = (MODE_TELEOP, MODE_AUTONOMOUS)

CTRL_FLAG_STOP           = 0x01
CTRL_FLAG_AUTONOMY_ABORT = 0x02
CTRL_FLAG_RETURN_REQUEST = 0x04
CTRL_INDICATOR_MASK      = 0x38
CTRL_INDICATOR_SHIFT     = 3
CTRL_FLAG_C2_LOST        = 0x40
CTRL_FLAG_FROM_AUTONOMY  = 0x80

SOURCE_TELEOP   = False
SOURCE_AUTONOMY = True

INDICATOR_OFF        = 0
INDICATOR_TELEOP     = 1
INDICATOR_AUTONOMOUS = 2
INDICATOR_ARRIVED    = 3
INDICATOR_FAULT      = 4
INDICATOR_MAX        = 7
INDICATOR_NAMES = {INDICATOR_OFF: "OFF", INDICATOR_TELEOP: "TELEOP",
                   INDICATOR_AUTONOMOUS: "AUTONOMOUS",
                   INDICATOR_ARRIVED: "ARRIVED", INDICATOR_FAULT: "FAULT"}

FAULT_COMM_TIMEOUT   = 0x0001
FAULT_OVER_CURRENT   = 0x0002
FAULT_ESTOP_ACTIVE   = 0x0004
FAULT_ENCODER_FAULT  = 0x0008
FAULT_UNDERVOLTAGE   = 0x0010
FAULT_FIRMWARE_FAULT = 0x0020
FAULT_PROTOCOL_ERROR = 0x0040
FAULT_C2_LINK_LOST   = 0x0080
FAULT_SEQ_GAP        = 0x0100
FAULT_CRC_ERROR      = 0x0200
FAULT_CMD_SOURCE     = 0x0400
FAULT_STALE_FEEDBACK = 0x0800
FAULT_STEERING_FAULT = 0x1000

FAULT_NAMES = [
    (FAULT_COMM_TIMEOUT, "COMM_TIMEOUT"),
    (FAULT_OVER_CURRENT, "OVER_CURRENT"),
    (FAULT_ESTOP_ACTIVE, "ESTOP_ACTIVE"),
    (FAULT_ENCODER_FAULT, "ENCODER_FAULT"),
    (FAULT_UNDERVOLTAGE, "UNDERVOLTAGE"),
    (FAULT_FIRMWARE_FAULT, "FIRMWARE_FAULT"),
    (FAULT_PROTOCOL_ERROR, "PROTOCOL_ERROR"),
    (FAULT_C2_LINK_LOST, "C2_LINK_LOST"),
    (FAULT_SEQ_GAP, "SEQ_GAP"),
    (FAULT_CRC_ERROR, "CRC_ERROR"),
    (FAULT_CMD_SOURCE, "CMD_SOURCE"),
    (FAULT_STALE_FEEDBACK, "STALE_FEEDBACK"),
    (FAULT_STEERING_FAULT, "STEERING_FAULT"),
]

FAULT_CRITICAL_MASK = FAULT_OVER_CURRENT | FAULT_ESTOP_ACTIVE

LINK_OK           = 0
LINK_DEGRADED     = 1
LINK_LOST         = 2
LINK_NOT_REPORTED = 3
LINK_NAMES = {LINK_OK: "OK", LINK_DEGRADED: "DEGRADED",
              LINK_LOST: "LOST", LINK_NOT_REPORTED: "NOT_REPORTED"}

CTRL_HEALTH_OK           = 0
CTRL_HEALTH_DEGRADED     = 1
CTRL_HEALTH_FAULT        = 2
CTRL_HEALTH_NOT_REPORTED = 3
CTRL_HEALTH_NAMES = {CTRL_HEALTH_OK: "OK", CTRL_HEALTH_DEGRADED: "DEGRADED",
                     CTRL_HEALTH_FAULT: "FAULT",
                     CTRL_HEALTH_NOT_REPORTED: "NOT_REPORTED"}

CMD_STATUS_APPLIED      = 0
CMD_STATUS_STOPPED      = 1
CMD_STATUS_WRONG_SOURCE = 2
CMD_STATUS_NO_COMMAND   = 3
CMD_STATUS_MODE_REFUSED = 4
CMD_STATUS_MAX          = 7
CMD_STATUS_NAMES = {CMD_STATUS_APPLIED: "APPLIED",
                    CMD_STATUS_STOPPED: "STOPPED",
                    CMD_STATUS_WRONG_SOURCE: "WRONG_SOURCE",
                    CMD_STATUS_NO_COMMAND: "NO_COMMAND",
                    CMD_STATUS_MODE_REFUSED: "MODE_REFUSED"}

CMD_ECHO_STATUS_MASK   = 0x07
CMD_ECHO_DECODE_MASK   = 0x38
CMD_ECHO_DECODE_SHIFT  = 3
CMD_ECHO_RETURN_ACTIVE = 0x40
CMD_ECHO_RESERVED_MASK = 0x80

CMD_AGE_UNKNOWN = 0xFFFF
CMD_AGE_MAX     = 0xFFFE

FEEDBACK_AGE_UNKNOWN = 0xFFFF
FEEDBACK_AGE_MAX     = 0xFFFE

CMD_SOURCE_NONE     = 0
CMD_SOURCE_OPERATOR = 1
CMD_SOURCE_AUTONOMY = 2
CMD_SOURCE_MAX      = 3
CMD_SOURCE_NAMES = {CMD_SOURCE_NONE: "NONE",
                    CMD_SOURCE_OPERATOR: "OPERATOR",
                    CMD_SOURCE_AUTONOMY: "AUTONOMY"}

FB_FLAG_ENCODER_STALE   = 0x01
FB_FLAG_STEER_STALE     = 0x02
FB_FLAG_ENCODER_INVALID = 0x04
FB_FLAG_STEER_INVALID   = 0x08
FB_FLAGS_MASK           = 0x0F
FB_FLAGS_RESERVED_MASK  = 0xF0
FB_FLAG_NAMES = ((FB_FLAG_ENCODER_STALE, "ENCODER_STALE"),
                 (FB_FLAG_STEER_STALE, "STEER_STALE"),
                 (FB_FLAG_ENCODER_INVALID, "ENCODER_INVALID"),
                 (FB_FLAG_STEER_INVALID, "STEER_INVALID"))


def feedback_flag_names(flags):
    return [name for bit, name in FB_FLAG_NAMES if flags & bit]

FRAME_DLC        = 8
FRAME_PAYLOAD    = 6
FRAME_SEQ_OFFSET = 6
FRAME_CRC_OFFSET = 7

CONTROL_FMT = "<hhBB"
DRIVE_L_FMT = "<ih"
DRIVE_R_FMT = "<iH"
POWER_FMT   = "<hhH"
STATE_FMT   = "<6B"
ECHO_FMT    = "<hhBB"
FEEDBACK_FMT = "<HHBB"

INT32_MIN = -(2 ** 31)
INT32_MAX = 2 ** 31 - 1

DECODE_OK            = 0
DECODE_BAD_DLC       = 1
DECODE_BAD_CRC       = 2
DECODE_BAD_MODE      = 3
DECODE_BAD_FLAGS     = 4
DECODE_BAD_INDICATOR = 5
DECODE_BAD_RANGE     = 6
DECODE_NAMES = {DECODE_OK: "OK", DECODE_BAD_DLC: "BAD_DLC",
                DECODE_BAD_CRC: "BAD_CRC", DECODE_BAD_MODE: "BAD_MODE",
                DECODE_BAD_FLAGS: "BAD_FLAGS",
                DECODE_BAD_INDICATOR: "BAD_INDICATOR",
                DECODE_BAD_RANGE: "BAD_RANGE"}


def fault_names(bitmask):
    return [name for bit, name in FAULT_NAMES if bitmask & bit]


def is_commandable_mode(mode):
    return mode in COMMANDABLE_MODES


def mode_permits_motion(mode):
    return mode in MOTION_MODES


def source_owns_mode(from_autonomy, mode):
    if not mode_permits_motion(mode):
        return True
    return bool(from_autonomy) == (mode == MODE_AUTONOMOUS)

_DENY, _ANYONE, _OPERATOR = range(3)
_TRANSITIONS = {
    MODE_SAFE:       (_ANYONE, _OPERATOR, _OPERATOR),
    MODE_TELEOP:     (_ANYONE, _ANYONE,   _OPERATOR),
    MODE_AUTONOMOUS: (_ANYONE, _OPERATOR, _ANYONE),
    MODE_FAULT:      (_ANYONE, _DENY,     _DENY),
}

def transition_allowed(from_mode, to_mode, from_autonomy):
    if from_mode not in _TRANSITIONS or not is_commandable_mode(to_mode):
        return False
    rule = _TRANSITIONS[from_mode][to_mode]
    if rule == _OPERATOR:
        return not from_autonomy
    return rule == _ANYONE

def is_valid_command(value):
    return CMD_MIN <= value <= CMD_MAX


def is_known_indicator(indicator):
    return 0 <= indicator <= INDICATOR_FAULT


def wrap_i32(value):
    return ((value - INT32_MIN) % (2 ** 32)) + INT32_MIN


def clamp_cmd_age_ms(age_ms):
    if age_ms < 0:
        return 0
    return age_ms if age_ms < CMD_AGE_MAX else CMD_AGE_MAX


def pack_cmd_status(status, decode=DECODE_OK, return_active=False):
    packed = status & CMD_ECHO_STATUS_MASK
    packed |= (decode << CMD_ECHO_DECODE_SHIFT) & CMD_ECHO_DECODE_MASK
    if return_active:
        packed |= CMD_ECHO_RETURN_ACTIVE
    return packed


def cmd_status_of(packed):
    return packed & CMD_ECHO_STATUS_MASK


def decode_result_of(packed):
    return (packed & CMD_ECHO_DECODE_MASK) >> CMD_ECHO_DECODE_SHIFT


def return_active_of(packed):
    return bool(packed & CMD_ECHO_RETURN_ACTIVE)


def seq_delta(previous, current):
    return (current - previous) & 0xFF


def crc8_update(crc, data):
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc8(data):
    return crc8_update(0xFF, data) ^ 0xFF


def frame_crc8(can_id, payload):
    id_bytes = bytes((can_id & 0xFF, (can_id >> 8) & 0xFF,
                      (can_id >> 16) & 0xFF, (can_id >> 24) & 0xFF))
    return crc8_update(crc8_update(0xFF, id_bytes), payload) ^ 0xFF


def _seal(can_id, payload, seq):
    assert len(payload) == FRAME_PAYLOAD, "payload must be 6 bytes"
    body = payload + bytes((seq & 0xFF,))
    return body + bytes((frame_crc8(can_id, body),))


def _open(can_id, frame):
    if len(frame) != FRAME_DLC:
        return DECODE_BAD_DLC, None, None
    body = frame[:FRAME_CRC_OFFSET]
    if frame_crc8(can_id, body) != frame[FRAME_CRC_OFFSET]:
        return DECODE_BAD_CRC, None, None
    return DECODE_OK, frame[:FRAME_PAYLOAD], frame[FRAME_SEQ_OFFSET]


def pack_control_flags(c2_lost, stop=False, autonomy_abort=False,
                       return_request=False, indicator_request=INDICATOR_OFF,
                       from_autonomy=False):
    flags = 0
    if stop:            flags |= CTRL_FLAG_STOP
    if autonomy_abort:  flags |= CTRL_FLAG_AUTONOMY_ABORT
    if return_request:  flags |= CTRL_FLAG_RETURN_REQUEST
    if c2_lost:         flags |= CTRL_FLAG_C2_LOST
    if from_autonomy:   flags |= CTRL_FLAG_FROM_AUTONOMY
    flags |= (indicator_request << CTRL_INDICATOR_SHIFT) & CTRL_INDICATOR_MASK
    return flags


def encode_control(drive_cmd, steer_cmd, mode, c2_lost, stop=False, seq=0,
                   autonomy_abort=False, return_request=False,
                   indicator_request=INDICATOR_OFF, from_autonomy=False,
                   raw_flags=None):
    flags = pack_control_flags(c2_lost, stop, autonomy_abort, return_request,
                               indicator_request,
                               from_autonomy) if raw_flags is None else raw_flags
    payload = struct.pack(CONTROL_FMT, drive_cmd, steer_cmd, mode, flags)
    return _seal(CAN_ID_CONTROL, payload, seq)


def decode_control(frame, can_id=CAN_ID_CONTROL):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    drive, steer, mode, flags = struct.unpack(CONTROL_FMT, payload)
    if not is_commandable_mode(mode):
        return DECODE_BAD_MODE, None
    indicator = (flags & CTRL_INDICATOR_MASK) >> CTRL_INDICATOR_SHIFT
    if not is_known_indicator(indicator):
        return DECODE_BAD_INDICATOR, None
    if not (is_valid_command(drive) and is_valid_command(steer)):
        return DECODE_BAD_RANGE, None
    return DECODE_OK, {
        "drive_cmd": drive, "steer_cmd": steer, "mode": mode,
        "stop": bool(flags & CTRL_FLAG_STOP),
        "autonomy_abort": bool(flags & CTRL_FLAG_AUTONOMY_ABORT),
        "return_request": bool(flags & CTRL_FLAG_RETURN_REQUEST),
        "c2_lost": bool(flags & CTRL_FLAG_C2_LOST),
        "from_autonomy": bool(flags & CTRL_FLAG_FROM_AUTONOMY),
        "indicator_request": indicator, "seq": seq,
    }


def encode_telemetry_drive_l(enc_left, steer_fb, seq=0):
    return _seal(CAN_ID_TELEM_DRIVE_L,
                 struct.pack(DRIVE_L_FMT, wrap_i32(enc_left), steer_fb), seq)


def encode_telemetry_drive_r(enc_right, cmd_age_ms, seq=0):
    return _seal(CAN_ID_TELEM_DRIVE_R,
                 struct.pack(DRIVE_R_FMT, wrap_i32(enc_right), cmd_age_ms), seq)


def encode_telemetry_power(current_ca, voltage_cv, fault_status, seq=0):
    return _seal(CAN_ID_TELEM_POWER,
                 struct.pack(POWER_FMT, current_ca, voltage_cv, fault_status), seq)


def encode_telemetry_state(mode, jetson_link, c2_link, controller_health,
                           indicator_state, seq=0, reserved=0):
    return _seal(CAN_ID_TELEM_STATE,
                 struct.pack(STATE_FMT, mode, jetson_link, c2_link,
                             controller_health, indicator_state, reserved), seq)


def decode_telemetry_drive_l(frame, can_id=CAN_ID_TELEM_DRIVE_L):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    enc_left, steer_fb = struct.unpack(DRIVE_L_FMT, payload)
    return DECODE_OK, {"enc_left": enc_left, "steer_fb": steer_fb, "seq": seq}


def decode_telemetry_drive_r(frame, can_id=CAN_ID_TELEM_DRIVE_R):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    enc_right, cmd_age = struct.unpack(DRIVE_R_FMT, payload)
    return DECODE_OK, {"enc_right": enc_right, "cmd_age_ms": cmd_age, "seq": seq}


def decode_telemetry_power(frame, can_id=CAN_ID_TELEM_POWER):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    current_ca, voltage_cv, faults = struct.unpack(POWER_FMT, payload)
    return DECODE_OK, {
        "current_ca": current_ca, "current_a": current_ca / 100.0,
        "voltage_cv": voltage_cv, "voltage_v": voltage_cv / 100.0,
        "fault_status": faults, "seq": seq,
    }


def decode_telemetry_state(frame, can_id=CAN_ID_TELEM_STATE):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    mode, jlink, c2link, health, indicator, reserved = struct.unpack(STATE_FMT, payload)
    return DECODE_OK, {
        "mode": mode, "jetson_link": jlink, "c2_link": c2link,
        "controller_health": health, "indicator_state": indicator,
        "reserved": reserved, "seq": seq,
    }


def encode_telemetry_cmd_echo(drive_applied, steer_applied, ack_seq, status,
                              seq=0):
    return _seal(CAN_ID_TELEM_CMD_ECHO,
                 struct.pack(ECHO_FMT, drive_applied, steer_applied,
                             ack_seq, status), seq)


def decode_telemetry_cmd_echo(frame, can_id=CAN_ID_TELEM_CMD_ECHO):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    drive, steer, ack_seq, status = struct.unpack(ECHO_FMT, payload)
    return DECODE_OK, {
        "drive_applied": drive, "steer_applied": steer,
        "ack_seq": ack_seq, "cmd_status_raw": status,
        "cmd_status": cmd_status_of(status),
        "last_decode": decode_result_of(status),
        "return_active": return_active_of(status),
        "seq": seq,
    }


def encode_telemetry_feedback(encoder_age_ms, steer_age_ms, cmd_source,
                              flags, seq=0):
    return _seal(CAN_ID_TELEM_FEEDBACK,
                 struct.pack(FEEDBACK_FMT, encoder_age_ms, steer_age_ms,
                             cmd_source, flags & FB_FLAGS_MASK), seq)


def decode_telemetry_feedback(frame, can_id=CAN_ID_TELEM_FEEDBACK):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    enc_age, steer_age, source, flags = struct.unpack(FEEDBACK_FMT, payload)
    flags &= FB_FLAGS_MASK
    return DECODE_OK, {
        "encoder_age_ms": enc_age, "steer_age_ms": steer_age,
        "cmd_source": source, "feedback_flags": flags,
        "encoder_stale": bool(flags & FB_FLAG_ENCODER_STALE),
        "steer_stale": bool(flags & FB_FLAG_STEER_STALE),
        "encoder_invalid": bool(flags & FB_FLAG_ENCODER_INVALID),
        "steer_invalid": bool(flags & FB_FLAG_STEER_INVALID),
        "seq": seq,
    }


DECODERS = {
    CAN_ID_TELEM_DRIVE_L: decode_telemetry_drive_l,
    CAN_ID_TELEM_DRIVE_R: decode_telemetry_drive_r,
    CAN_ID_TELEM_POWER:   decode_telemetry_power,
    CAN_ID_TELEM_STATE:   decode_telemetry_state,
    CAN_ID_TELEM_CMD_ECHO: decode_telemetry_cmd_echo,
    CAN_ID_TELEM_FEEDBACK: decode_telemetry_feedback,
}
