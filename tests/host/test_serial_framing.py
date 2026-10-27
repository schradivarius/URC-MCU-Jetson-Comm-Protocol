"""
test_serial_framing.py -- pin host/serial_link.py to firmware/src/rover_serial.*

The framing exists twice, in Python on the laptop and in C++ on the Teensy,
for the same reason the protocol codec does. This file is what stops the two
drifting:

  * PINNED_FRAME_HEX is the same literal as PINNED_HEX in
    tests/cpp/test_serial.cpp. Both sides encode the same (id, payload) and
    must produce those exact bytes, so neither can change the wire format
    alone.
  * The SLIP byte values and buffer sizes are read out of the C++ header as
    text, the way test_config_sync.py reads rover_config.h, so no toolchain is
    needed to catch a one-sided edit.
  * A CONTROL frame built by host/rover_protocol.py is pushed through the
    framer and decoded again, which proves the two layers compose: framing
    does not disturb the payload the CRC is computed over.
"""

import os
import re
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import protocol_root  # noqa: E402
import rover_protocol as rp  # noqa: E402
import serial_link as sl  # noqa: E402

HEADER_PATH = protocol_root.firmware_src("rover_serial.h")

PINNED_PAYLOAD = bytes([0xC0, 0xDB, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05])
PINNED_FRAME_HEX = "c00001dbdcdbdd000102030405c0"


def cpp_constants():
    with open(HEADER_PATH) as f:
        text = re.sub(r"//[^\n]*", "", f.read())
    found = {}
    for name, value in re.findall(
            r"\b([A-Z][A-Z0-9_]*)\s*=\s*(0[xX][0-9A-Fa-f]+|\d+)\b", text):
        found[name] = int(value, 0)
    return found


class TestFramingMatchesTheFirmware(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpp = cpp_constants()

    def test_the_header_was_parsed(self):
        self.assertIn("SLIP_END", self.cpp, "rover_serial.h was not parsed")

    def test_the_slip_byte_values_match(self):
        for name in ("SLIP_END", "SLIP_ESC", "SLIP_ESC_END", "SLIP_ESC_ESC"):
            self.assertEqual(getattr(sl, name), self.cpp[name],
                             "%s differs between the two framers" % name)

    def test_the_buffer_sizes_match(self):
        self.assertEqual(sl.SERIAL_ID_BYTES, self.cpp["SERIAL_ID_BYTES"])
        self.assertEqual(sl.FRAME_DLC, rp.FRAME_DLC)
        self.assertEqual(sl.SERIAL_BODY_MAX,
                         sl.SERIAL_ID_BYTES + sl.FRAME_DLC)
        self.assertEqual(sl.SERIAL_ENCODED_MAX, 2 + 2 * sl.SERIAL_BODY_MAX)

    def test_the_escape_codes_are_distinct_from_the_bytes_they_escape(self):
        # If an escape code ever equalled END or ESC the framing would be
        # unparseable, and the symptom would be a link that works until a
        # payload happens to contain one byte.
        self.assertEqual(len({sl.SLIP_END, sl.SLIP_ESC, sl.SLIP_ESC_END,
                              sl.SLIP_ESC_ESC}), 4)

    def test_the_wire_format_is_pinned(self):
        frame = sl.encode_frame(rp.CAN_ID_CONTROL, PINNED_PAYLOAD)
        self.assertEqual(frame.hex(), PINNED_FRAME_HEX,
                         "the Python framer no longer agrees with the literal "
                         "pinned in tests/cpp/test_serial.cpp")

    def test_the_encoded_worst_case_fits_the_declared_buffer(self):
        frame = sl.encode_frame(0x7FF, bytes([sl.SLIP_END] * sl.FRAME_DLC))
        self.assertLessEqual(len(frame), sl.SERIAL_ENCODED_MAX)


class TestFramingBehaviour(unittest.TestCase):
    def round_trip(self, can_id, payload):
        reader = sl.FrameReader()
        frames = reader.feed(sl.encode_frame(can_id, payload))
        self.assertEqual(frames, [(can_id, bytes(payload))])
        self.assertEqual(reader.dropped, 0)
        return reader

    def test_every_protocol_id_round_trips(self):
        for can_id in (rp.CAN_ID_CONTROL,) + rp.TELEM_IDS:
            self.round_trip(can_id, PINNED_PAYLOAD)

    def test_every_byte_value_round_trips(self):
        for value in range(256):
            self.round_trip(rp.CAN_ID_CONTROL, bytes([value]) * sl.FRAME_DLC)

    def test_frames_split_across_reads_still_arrive(self):
        frame = sl.encode_frame(rp.CAN_ID_TELEM_POWER, PINNED_PAYLOAD)
        for cut in range(1, len(frame)):
            reader = sl.FrameReader()
            got = reader.feed(frame[:cut]) + reader.feed(frame[cut:])
            self.assertEqual(got, [(rp.CAN_ID_TELEM_POWER, PINNED_PAYLOAD)],
                             "a frame cut at byte %d was lost" % cut)

    def test_a_whole_telemetry_cycle_arrives_in_order(self):
        stream = b""
        for can_id in rp.TELEM_IDS:
            stream += sl.encode_frame(can_id, PINNED_PAYLOAD)
        reader = sl.FrameReader()
        self.assertEqual([can_id for can_id, _ in reader.feed(stream)],
                         list(rp.TELEM_IDS))
        self.assertEqual(reader.dropped, 0)

    def test_leading_noise_costs_one_frame_and_no_more(self):
        reader = sl.FrameReader()
        frames = reader.feed(b"rover firmware up\r\n"
                             + sl.encode_frame(rp.CAN_ID_CONTROL, PINNED_PAYLOAD))
        self.assertEqual(frames, [(rp.CAN_ID_CONTROL, PINNED_PAYLOAD)])
        self.assertEqual(reader.dropped, 1)

    def test_a_board_reset_mid_frame_costs_one_frame(self):
        frame = sl.encode_frame(rp.CAN_ID_CONTROL, PINNED_PAYLOAD)
        reader = sl.FrameReader()
        frames = reader.feed(frame[:4] + frame)
        self.assertEqual(frames, [(rp.CAN_ID_CONTROL, PINNED_PAYLOAD)])
        self.assertEqual(reader.dropped, 1)

    def test_an_empty_body_is_not_a_drop(self):
        reader = sl.FrameReader()
        self.assertEqual(reader.feed(bytes([sl.SLIP_END] * 6)), [])
        self.assertEqual(reader.dropped, 0)

    def test_an_overlong_body_is_dropped(self):
        reader = sl.FrameReader()
        body = bytes([0x11]) * (sl.SERIAL_BODY_MAX + 4)
        self.assertEqual(reader.feed(bytes([sl.SLIP_END]) + body
                                     + bytes([sl.SLIP_END])), [])
        self.assertEqual(reader.dropped, 1)

    def test_an_id_only_body_is_dropped(self):
        reader = sl.FrameReader()
        self.assertEqual(reader.feed(bytes([sl.SLIP_END, 0x00, 0x01,
                                            sl.SLIP_END])), [])
        self.assertEqual(reader.dropped, 1)

    def test_an_undefined_escape_code_is_dropped(self):
        reader = sl.FrameReader()
        self.assertEqual(reader.feed(bytes([sl.SLIP_END, 0x00, 0x01,
                                            sl.SLIP_ESC, 0x41, 0x02,
                                            sl.SLIP_END])), [])
        self.assertEqual(reader.dropped, 1)

    def test_an_id_wider_than_eleven_bits_is_refused_both_ways(self):
        with self.assertRaises(ValueError):
            sl.encode_frame(0x800, PINNED_PAYLOAD)
        reader = sl.FrameReader()
        self.assertEqual(reader.feed(bytes([sl.SLIP_END, 0xFF, 0xFF, 0x01,
                                            sl.SLIP_END])), [])
        self.assertEqual(reader.dropped, 1)

    def test_an_oversized_payload_is_refused(self):
        with self.assertRaises(ValueError):
            sl.encode_frame(rp.CAN_ID_CONTROL, bytes(sl.FRAME_DLC + 1))


class TestFramingComposesWithTheProtocol(unittest.TestCase):
    def test_a_control_frame_survives_framing_and_still_validates(self):
        payload = rp.encode_control(450, -320, rp.MODE_TELEOP, c2_lost=False,
                                    seq=0xC0,
                                    indicator_request=rp.INDICATOR_TELEOP)
        reader = sl.FrameReader()
        frames = reader.feed(sl.encode_frame(rp.CAN_ID_CONTROL, payload))
        self.assertEqual(len(frames), 1)
        can_id, received = frames[0]

        result, msg = rp.decode_control(received, can_id=can_id)
        self.assertEqual(result, rp.DECODE_OK, rp.DECODE_NAMES.get(result))
        self.assertEqual(msg["drive_cmd"], 450)
        self.assertEqual(msg["steer_cmd"], -320)
        self.assertEqual(msg["mode"], rp.MODE_TELEOP)
        self.assertEqual(msg["seq"], 0xC0)

    def test_a_frame_whose_payload_is_all_framing_bytes_still_validates(self):
        payload = rp.encode_control(-1000, 1000, rp.MODE_TELEOP, c2_lost=True,
                                    seq=sl.SLIP_ESC)
        reader = sl.FrameReader()
        frames = reader.feed(sl.encode_frame(rp.CAN_ID_CONTROL, payload))
        self.assertEqual(len(frames), 1)
        result, msg = rp.decode_control(frames[0][1])
        self.assertEqual(result, rp.DECODE_OK)
        self.assertEqual(msg["seq"], sl.SLIP_ESC)

    def test_a_corrupted_payload_is_caught_by_the_protocol_crc(self):
        payload = bytearray(rp.encode_control(100, 0, rp.MODE_TELEOP,
                                              c2_lost=False, seq=7))
        payload[0] ^= 0x01
        reader = sl.FrameReader()
        frames = reader.feed(sl.encode_frame(rp.CAN_ID_CONTROL, bytes(payload)))
        self.assertEqual(len(frames), 1)
        result, msg = rp.decode_control(frames[0][1])
        self.assertEqual(result, rp.DECODE_BAD_CRC)
        self.assertIsNone(msg)

    def test_every_telemetry_frame_round_trips_through_the_framer(self):
        built = {
            rp.CAN_ID_TELEM_DRIVE_L: rp.encode_telemetry_drive_l(-12345, 250, seq=9),
            rp.CAN_ID_TELEM_DRIVE_R: rp.encode_telemetry_drive_r(98765, 40, seq=9),
            rp.CAN_ID_TELEM_POWER: rp.encode_telemetry_power(1500, 2400, 0, seq=9),
            rp.CAN_ID_TELEM_STATE: rp.encode_telemetry_state(
                rp.MODE_TELEOP, rp.LINK_OK, rp.LINK_NOT_REPORTED,
                rp.CTRL_HEALTH_OK, rp.INDICATOR_TELEOP, seq=9),
            rp.CAN_ID_TELEM_CMD_ECHO: rp.encode_telemetry_cmd_echo(
                450, -320, 0xC0, rp.pack_cmd_status(rp.CMD_STATUS_APPLIED), seq=9),
        }
        stream = b""
        for can_id, payload in built.items():
            stream += sl.encode_frame(can_id, payload)

        reader = sl.FrameReader()
        frames = reader.feed(stream)
        self.assertEqual(len(frames), len(built))
        for can_id, received in frames:
            result, msg = rp.DECODERS[can_id](received)
            self.assertEqual(result, rp.DECODE_OK,
                             "%#x: %s" % (can_id, rp.DECODE_NAMES.get(result)))
            self.assertEqual(msg["seq"], 9)
            self.assertEqual(received, built[can_id])


if __name__ == "__main__":
    unittest.main()
