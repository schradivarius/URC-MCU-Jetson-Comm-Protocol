#include "rover_serial.h"

namespace rover {

namespace {

bool putStuffed(uint8_t byte, uint8_t* out, size_t cap, size_t& at) {
    if (byte == SLIP_END || byte == SLIP_ESC) {
        if (at + 2 > cap) return false;
        out[at++] = SLIP_ESC;
        out[at++] = (byte == SLIP_END) ? SLIP_ESC_END : SLIP_ESC_ESC;
        return true;
    }
    if (at + 1 > cap) return false;
    out[at++] = byte;
    return true;
}

}  // namespace

size_t encodeSerialFrame(uint32_t id, const uint8_t* payload, size_t len,
                         uint8_t* out, size_t cap) {
    if (out == nullptr || len > FRAME_DLC) return 0;
    if (len > 0 && payload == nullptr) return 0;
    if (id > 0x7FFu) return 0;

    size_t at = 0;
    if (at + 1 > cap) return 0;
    out[at++] = SLIP_END;

    const uint8_t header[SERIAL_ID_BYTES] = {
        static_cast<uint8_t>(id & 0xFFu),
        static_cast<uint8_t>((id >> 8) & 0xFFu),
    };
    for (size_t i = 0; i < SERIAL_ID_BYTES; ++i) {
        if (!putStuffed(header[i], out, cap, at)) return 0;
    }
    for (size_t i = 0; i < len; ++i) {
        if (!putStuffed(payload[i], out, cap, at)) return 0;
    }

    if (at + 1 > cap) return 0;
    out[at++] = SLIP_END;
    return at;
}

SerialFrameReader::SerialFrameReader() { reset(); }

void SerialFrameReader::reset() {
    body_len_ = 0;
    payload_len_ = 0;
    id_ = 0;
    escaped_ = false;
    bad_ = false;
    accepted_ = 0;
    dropped_ = 0;
}

bool SerialFrameReader::finish() {
    const uint32_t id = body_len_ >= SERIAL_ID_BYTES
        ? (static_cast<uint32_t>(body_[0]) |
           (static_cast<uint32_t>(body_[1]) << 8))
        : 0u;
    const bool bad = bad_ || body_len_ <= SERIAL_ID_BYTES || id > 0x7FFu;
    const size_t len = bad ? 0 : body_len_ - SERIAL_ID_BYTES;

    if (bad) {
        if (body_len_ != 0 || bad_) ++dropped_;
        body_len_ = 0;
        bad_ = false;
        escaped_ = false;
        return false;
    }

    id_ = id;
    payload_len_ = len;
    for (size_t i = 0; i < len; ++i) payload_[i] = body_[SERIAL_ID_BYTES + i];

    ++accepted_;
    body_len_ = 0;
    escaped_ = false;
    return true;
}

bool SerialFrameReader::feed(uint8_t byte) {
    if (byte == SLIP_END) return finish();

    if (escaped_) {
        escaped_ = false;
        if (byte == SLIP_ESC_END)      byte = SLIP_END;
        else if (byte == SLIP_ESC_ESC) byte = SLIP_ESC;
        else {
            bad_ = true;
            return false;
        }
    } else if (byte == SLIP_ESC) {
        escaped_ = true;
        return false;
    }

    if (body_len_ >= SERIAL_BODY_MAX) {
        bad_ = true;
        return false;
    }
    body_[body_len_++] = byte;
    return false;
}

}  // namespace rover
