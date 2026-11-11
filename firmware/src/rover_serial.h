#ifndef ROVER_SERIAL_H
#define ROVER_SERIAL_H

#include <stddef.h>
#include <stdint.h>

#include "rover_config.h"

namespace rover {

enum : uint8_t {
    SLIP_END     = 0xC0,
    SLIP_ESC     = 0xDB,
    SLIP_ESC_END = 0xDC,
    SLIP_ESC_ESC = 0xDD,
};

enum : size_t {
    SERIAL_ID_BYTES    = 2,
    SERIAL_BODY_MAX    = SERIAL_ID_BYTES + FRAME_DLC,
    SERIAL_ENCODED_MAX = 2 + 2 * SERIAL_BODY_MAX,
};

size_t encodeSerialFrame(uint32_t id, const uint8_t* payload, size_t len,
                         uint8_t* out, size_t cap);

class SerialFrameReader {
public:
    SerialFrameReader();

    bool feed(uint8_t byte);

    uint32_t       frameId() const { return id_; }
    const uint8_t* framePayload() const { return payload_; }
    size_t         framePayloadLen() const { return payload_len_; }

    uint32_t framesAccepted() const { return accepted_; }
    uint32_t framesDropped() const { return dropped_; }

    void reset();

private:
    bool finish();

    uint8_t  body_[SERIAL_BODY_MAX];
    uint8_t  payload_[FRAME_DLC];
    size_t   body_len_;
    size_t   payload_len_;
    uint32_t id_;
    bool     escaped_;
    bool     bad_;
    uint32_t accepted_;
    uint32_t dropped_;
};

}  // namespace rover

#endif  // ROVER_SERIAL_H
