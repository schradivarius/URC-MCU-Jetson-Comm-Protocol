#include <cstring>
#include <string>
#include <vector>

#include "../../firmware/src/rover_protocol.h"
#include "../../firmware/src/rover_serial.h"
#include "test_framework.h"

using namespace rover;

static const char* PINNED_HEX = "c00001dbdcdbdd000102030405c0";
static const uint8_t PINNED_PAYLOAD[FRAME_DLC] = {
    0xC0, 0xDB, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

static std::string toHex(const uint8_t* data, size_t len) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    for (size_t i = 0; i < len; ++i) {
        out += digits[data[i] >> 4];
        out += digits[data[i] & 0x0F];
    }
    return out;
}

static std::vector<uint8_t> encode(uint32_t id, const uint8_t* payload,
                                   size_t len) {
    uint8_t buf[SERIAL_ENCODED_MAX];
    const size_t n = encodeSerialFrame(id, payload, len, buf, sizeof(buf));
    return std::vector<uint8_t>(buf, buf + n);
}

static size_t feedAll(SerialFrameReader& reader, const std::vector<uint8_t>& bytes,
                      uint32_t* last_id = nullptr,
                      std::vector<uint8_t>* last_payload = nullptr) {
    size_t frames = 0;
    for (uint8_t byte : bytes) {
        if (reader.feed(byte)) {
            ++frames;
            if (last_id) *last_id = reader.frameId();
            if (last_payload) {
                last_payload->assign(reader.framePayload(),
                                     reader.framePayload() + reader.framePayloadLen());
            }
        }
    }
    return frames;
}

static void the_wire_format_is_pinned() {
    const std::vector<uint8_t> frame =
        encode(CAN_ID_CONTROL, PINNED_PAYLOAD, FRAME_DLC);
    CHECK_STREQ(toHex(frame.data(), frame.size()).c_str(), PINNED_HEX);
}

static void both_framing_bytes_are_escaped() {
    const std::vector<uint8_t> frame =
        encode(CAN_ID_CONTROL, PINNED_PAYLOAD, FRAME_DLC);
    for (size_t i = 1; i + 1 < frame.size(); ++i) {
        CHECK(frame[i] != SLIP_END);
    }
}

static void a_frame_round_trips() {
    uint8_t payload[FRAME_DLC];
    for (size_t i = 0; i < FRAME_DLC; ++i) payload[i] = static_cast<uint8_t>(i * 37);

    SerialFrameReader reader;
    uint32_t id = 0;
    std::vector<uint8_t> out;
    CHECK_EQ(feedAll(reader, encode(CAN_ID_TELEM_POWER, payload, FRAME_DLC),
                     &id, &out), (size_t)1);
    CHECK_EQ(id, (uint32_t)CAN_ID_TELEM_POWER);
    CHECK_EQ(out.size(), (size_t)FRAME_DLC);
    CHECK(std::memcmp(out.data(), payload, FRAME_DLC) == 0);
    CHECK_EQ(reader.framesDropped(), (uint32_t)0);
}

static void every_protocol_id_round_trips() {
    const uint32_t ids[] = {CAN_ID_CONTROL, CAN_ID_TELEM_DRIVE_L,
                            CAN_ID_TELEM_DRIVE_R, CAN_ID_TELEM_POWER,
                            CAN_ID_TELEM_STATE, CAN_ID_TELEM_CMD_ECHO};
    for (uint32_t want : ids) {
        SerialFrameReader reader;
        uint32_t got = 0;
        CHECK_EQ(feedAll(reader, encode(want, PINNED_PAYLOAD, FRAME_DLC), &got),
                 (size_t)1);
        CHECK_EQ(got, want);
    }
}

static void back_to_back_frames_both_arrive() {
    std::vector<uint8_t> stream = encode(CAN_ID_TELEM_STATE, PINNED_PAYLOAD, FRAME_DLC);
    const std::vector<uint8_t> second =
        encode(CAN_ID_TELEM_POWER, PINNED_PAYLOAD, FRAME_DLC);
    stream.insert(stream.end(), second.begin(), second.end());

    SerialFrameReader reader;
    uint32_t last = 0;
    CHECK_EQ(feedAll(reader, stream, &last), (size_t)2);
    CHECK_EQ(last, (uint32_t)CAN_ID_TELEM_POWER);
    CHECK_EQ(reader.framesDropped(), (uint32_t)0);
    CHECK_EQ(reader.framesAccepted(), (uint32_t)2);
}

static void leading_noise_costs_one_frame_and_no_more() {
    const char* noise = "rover firmware up\r\n";
    std::vector<uint8_t> stream(noise, noise + std::strlen(noise));
    const std::vector<uint8_t> frame =
        encode(CAN_ID_CONTROL, PINNED_PAYLOAD, FRAME_DLC);
    stream.insert(stream.end(), frame.begin(), frame.end());

    SerialFrameReader reader;
    uint32_t id = 0;
    CHECK_EQ(feedAll(reader, stream, &id), (size_t)1);
    CHECK_EQ(id, (uint32_t)CAN_ID_CONTROL);
    CHECK_EQ(reader.framesDropped(), (uint32_t)1);
}

static void a_truncated_frame_does_not_eat_the_next_one() {
    const std::vector<uint8_t> frame =
        encode(CAN_ID_CONTROL, PINNED_PAYLOAD, FRAME_DLC);
    std::vector<uint8_t> stream(frame.begin(), frame.begin() + 4);
    stream.insert(stream.end(), frame.begin(), frame.end());

    SerialFrameReader reader;
    uint32_t id = 0;
    std::vector<uint8_t> out;
    CHECK_EQ(feedAll(reader, stream, &id, &out), (size_t)1);
    CHECK_EQ(id, (uint32_t)CAN_ID_CONTROL);
    CHECK_EQ(out.size(), (size_t)FRAME_DLC);
    CHECK_EQ(reader.framesDropped(), (uint32_t)1);
}

static void an_overlong_body_is_dropped() {
    std::vector<uint8_t> stream;
    stream.push_back(SLIP_END);
    for (size_t i = 0; i < SERIAL_BODY_MAX + 4; ++i) stream.push_back(0x11);
    stream.push_back(SLIP_END);

    SerialFrameReader reader;
    CHECK_EQ(feedAll(reader, stream), (size_t)0);
    CHECK_EQ(reader.framesDropped(), (uint32_t)1);
}

static void an_empty_body_is_not_counted_as_a_drop() {
    SerialFrameReader reader;
    for (int i = 0; i < 5; ++i) CHECK(!reader.feed(SLIP_END));
    CHECK_EQ(reader.framesDropped(), (uint32_t)0);
}

static void an_id_only_body_is_dropped() {
    std::vector<uint8_t> stream = {SLIP_END, 0x00, 0x01, SLIP_END};
    SerialFrameReader reader;
    CHECK_EQ(feedAll(reader, stream), (size_t)0);
    CHECK_EQ(reader.framesDropped(), (uint32_t)1);
}

static void an_undefined_escape_code_is_dropped() {
    std::vector<uint8_t> stream = {SLIP_END, 0x00, 0x01, SLIP_ESC, 0x41,
                                   0x02, SLIP_END};
    SerialFrameReader reader;
    CHECK_EQ(feedAll(reader, stream), (size_t)0);
    CHECK_EQ(reader.framesDropped(), (uint32_t)1);
}

static void an_id_wider_than_eleven_bits_is_refused() {
    uint8_t buf[SERIAL_ENCODED_MAX];
    CHECK_EQ(encodeSerialFrame(0x800, PINNED_PAYLOAD, FRAME_DLC, buf, sizeof(buf)),
             (size_t)0);

    std::vector<uint8_t> stream = {SLIP_END, 0xFF, 0xFF, 0x01, SLIP_END};
    SerialFrameReader reader;
    CHECK_EQ(feedAll(reader, stream), (size_t)0);
    CHECK_EQ(reader.framesDropped(), (uint32_t)1);
}

static void an_oversized_payload_is_refused() {
    uint8_t payload[FRAME_DLC + 1] = {0};
    uint8_t buf[SERIAL_ENCODED_MAX];
    CHECK_EQ(encodeSerialFrame(CAN_ID_CONTROL, payload, sizeof(payload), buf,
                               sizeof(buf)), (size_t)0);
}

static void a_buffer_too_small_is_refused_rather_than_overrun() {
    uint8_t buf[SERIAL_ENCODED_MAX];
    const size_t needed = encodeSerialFrame(CAN_ID_CONTROL, PINNED_PAYLOAD,
                                            FRAME_DLC, buf, sizeof(buf));
    CHECK(needed > 0);
    for (size_t cap = 0; cap < needed; ++cap) {
        std::vector<uint8_t> small(needed, 0);
        CHECK_EQ(encodeSerialFrame(CAN_ID_CONTROL, PINNED_PAYLOAD, FRAME_DLC,
                                   small.data(), cap), (size_t)0);
    }
}

static void the_worst_case_frame_fits_the_declared_buffer() {
    uint8_t payload[FRAME_DLC];
    for (size_t i = 0; i < FRAME_DLC; ++i) payload[i] = SLIP_END;
    uint8_t buf[SERIAL_ENCODED_MAX];
    const size_t n = encodeSerialFrame(0x7FF, payload, FRAME_DLC, buf, sizeof(buf));
    CHECK(n > 0);
    CHECK(n <= SERIAL_ENCODED_MAX);
}

static void a_control_frame_survives_framing_and_still_validates() {
    ControlMsg sent;
    sent.drive_cmd = 450;
    sent.steer_cmd = -320;
    sent.mode = MODE_TELEOP;
    sent.stop = false;
    sent.autonomy_abort = false;
    sent.return_request = false;
    sent.indicator_request = INDICATOR_TELEOP;
    sent.c2_lost = false;
    sent.from_autonomy = false;
    sent.seq = 0xC0;

    uint8_t payload[FRAME_DLC];
    const uint8_t len = encodeControl(sent, payload);

    SerialFrameReader reader;
    uint32_t id = 0;
    std::vector<uint8_t> out;
    CHECK_EQ(feedAll(reader, encode(CAN_ID_CONTROL, payload, len), &id, &out),
             (size_t)1);

    ControlMsg got;
    CHECK_EQ(decodeControl(id, out.data(), static_cast<uint8_t>(out.size()), got),
             DECODE_OK);
    CHECK_EQ(got.drive_cmd, sent.drive_cmd);
    CHECK_EQ(got.steer_cmd, sent.steer_cmd);
    CHECK_EQ(got.mode, sent.mode);
    CHECK_EQ(got.seq, sent.seq);
}

static void a_corrupted_payload_is_caught_by_the_protocol_crc() {
    ControlMsg sent;
    sent.drive_cmd = 100; sent.steer_cmd = 0; sent.mode = MODE_TELEOP;
    sent.stop = false; sent.autonomy_abort = false; sent.return_request = false;
    sent.indicator_request = INDICATOR_OFF; sent.c2_lost = false;
    sent.from_autonomy = false; sent.seq = 7;

    uint8_t payload[FRAME_DLC];
    const uint8_t len = encodeControl(sent, payload);
    payload[0] = static_cast<uint8_t>(payload[0] ^ 0x01);

    SerialFrameReader reader;
    uint32_t id = 0;
    std::vector<uint8_t> out;
    CHECK_EQ(feedAll(reader, encode(CAN_ID_CONTROL, payload, len), &id, &out),
             (size_t)1);

    ControlMsg got;
    CHECK_EQ(decodeControl(id, out.data(), static_cast<uint8_t>(out.size()), got),
             DECODE_BAD_CRC);
}

int main() {
    RUN_TEST(the_wire_format_is_pinned);
    RUN_TEST(both_framing_bytes_are_escaped);
    RUN_TEST(a_frame_round_trips);
    RUN_TEST(every_protocol_id_round_trips);
    RUN_TEST(back_to_back_frames_both_arrive);
    RUN_TEST(leading_noise_costs_one_frame_and_no_more);
    RUN_TEST(a_truncated_frame_does_not_eat_the_next_one);
    RUN_TEST(an_overlong_body_is_dropped);
    RUN_TEST(an_empty_body_is_not_counted_as_a_drop);
    RUN_TEST(an_id_only_body_is_dropped);
    RUN_TEST(an_undefined_escape_code_is_dropped);
    RUN_TEST(an_id_wider_than_eleven_bits_is_refused);
    RUN_TEST(an_oversized_payload_is_refused);
    RUN_TEST(a_buffer_too_small_is_refused_rather_than_overrun);
    RUN_TEST(the_worst_case_frame_fits_the_declared_buffer);
    RUN_TEST(a_control_frame_survives_framing_and_still_validates);
    RUN_TEST(a_corrupted_payload_is_caught_by_the_protocol_crc);
    return testing::summary("test_serial");
}
