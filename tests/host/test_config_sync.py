import os
import re
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import c2_link  # noqa: E402
import protocol_root  # noqa: E402
import rover_config as cfg  # noqa: E402
import rover_protocol as rp  # noqa: E402

HEADER_PATH = protocol_root.firmware_src("rover_config.h")

SHARED = {
    "PROTOCOL_VERSION":      "PROTOCOL_VERSION",
    "SERIAL_BAUD_HZ":        "SERIAL_BAUD_HZ",
    "WATCHDOG_TIMEOUT_MS":   "DEFAULT_WATCHDOG_TIMEOUT_MS",
    "TELEMETRY_PERIOD_MS":   "DEFAULT_TELEMETRY_PERIOD_MS",
    "LINK_DEGRADED_HOLD_MS": "LINK_DEGRADED_HOLD_MS",
    "CAN_ID_CONTROL":        "CAN_ID_CONTROL",
    "CAN_ID_TELEM_DRIVE_L":  "CAN_ID_TELEM_DRIVE_L",
    "CAN_ID_TELEM_DRIVE_R":  "CAN_ID_TELEM_DRIVE_R",
    "CAN_ID_TELEM_POWER":    "CAN_ID_TELEM_POWER",
    "CAN_ID_TELEM_STATE":    "CAN_ID_TELEM_STATE",
    "CAN_ID_TELEM_CMD_ECHO": "CAN_ID_TELEM_CMD_ECHO",
    "CAN_ID_TELEM_FEEDBACK": "CAN_ID_TELEM_FEEDBACK",
    "FEEDBACK_STALE_MS":     "FEEDBACK_STALE_MS",
    "MODE_SAFE":             "MODE_SAFE",
    "MODE_TELEOP":           "MODE_TELEOP",
    "MODE_AUTONOMOUS":       "MODE_AUTONOMOUS",
    "MODE_FAULT":            "MODE_FAULT",
    "CMD_MIN":               "CMD_MIN",
    "CMD_MAX":               "CMD_MAX",
    "PWM_NEUTRAL_US":        "PWM_NEUTRAL_US",
    "PWM_SPAN_US":           "PWM_SPAN_US",
}

FRAME_GEOMETRY = ("FRAME_DLC", "FRAME_PAYLOAD", "FRAME_SEQ_OFFSET",
                  "FRAME_CRC_OFFSET")

FIRMWARE_ONLY = ("HW_WATCHDOG_MS", "OVER_CURRENT_CA", "UNDERVOLTAGE_CV")


def cpp_constants():
    with open(HEADER_PATH) as f:
        text = re.sub(r"//[^\n]*", "", f.read())
    found = {}
    for name, value in re.findall(
            r"\b([A-Z][A-Z0-9_]*)\s*=\s*(-?\s*(?:0[xX][0-9A-Fa-f]+|\d+))\b", text):
        found[name] = int(value.replace(" ", ""), 0)
    return found


class TestConfigSync(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpp = cpp_constants()

    def test_header_was_parsed(self):
        for cpp_name in SHARED.values():
            self.assertIn(cpp_name, self.cpp,
                          "%s not found in rover_config.h" % cpp_name)

    def test_python_config_matches_cpp_config(self):
        for py_name, cpp_name in SHARED.items():
            self.assertEqual(
                getattr(cfg, py_name), self.cpp[cpp_name],
                "%s differs: rover_config.py=%r, rover_config.h=%r -- the "
                "Jetson and the Teensy disagree"
                % (py_name, getattr(cfg, py_name), self.cpp[cpp_name]))

    def test_frame_geometry_matches_cpp(self):
        for name in FRAME_GEOMETRY:
            self.assertEqual(getattr(rp, name), self.cpp[name],
                             "%s differs between the two sides" % name)

    def test_payload_layouts_fit_the_frame(self):
        import struct
        for name in ("CONTROL_FMT", "DRIVE_L_FMT", "DRIVE_R_FMT",
                     "POWER_FMT", "STATE_FMT", "ECHO_FMT"):
            self.assertEqual(struct.calcsize(getattr(rp, name)),
                             self.cpp["FRAME_PAYLOAD"],
                             "%s is not FRAME_PAYLOAD bytes" % name)
        self.assertEqual(self.cpp["FRAME_PAYLOAD"] + 2, self.cpp["FRAME_DLC"])

    def test_firmware_only_constants_are_in_the_config_header(self):
        for name in FIRMWARE_ONLY:
            self.assertIn(name, self.cpp,
                          "%s belongs in rover_config.h" % name)

    def test_mode_max_is_the_highest_commandable_mode(self):
        self.assertEqual(cfg.MODE_MAX, max(rp.COMMANDABLE_MODES))

    def test_link_timeout_is_longer_than_a_telemetry_period(self):
        self.assertGreater(cfg.LINK_TIMEOUT_S * 1000, cfg.TELEMETRY_PERIOD_MS)

    def test_watchdog_tolerates_a_missed_control_frame(self):
        self.assertGreater(cfg.WATCHDOG_TIMEOUT_MS, 2 * 1000 // cfg.CONTROL_RATE_HZ)

    def test_c2_timeout_is_longer_than_the_command_watchdog(self):
        self.assertGreater(cfg.C2_TIMEOUT_S * 1000, cfg.WATCHDOG_TIMEOUT_MS)

    def test_c2_monitor_defaults_to_the_configured_timeout(self):
        self.assertEqual(c2_link.C2Monitor().C2_timeout_s, cfg.C2_TIMEOUT_S)

    def test_degraded_hold_outlasts_a_telemetry_period(self):
        self.assertGreater(cfg.LINK_DEGRADED_HOLD_MS, cfg.TELEMETRY_PERIOD_MS)

    def test_feedback_staleness_sits_between_the_two_timeouts(self):
        self.assertGreater(cfg.FEEDBACK_STALE_MS, cfg.TELEMETRY_PERIOD_MS)
        self.assertLess(cfg.FEEDBACK_STALE_MS, cfg.WATCHDOG_TIMEOUT_MS)

    def test_command_range_is_symmetric_and_nonzero(self):
        self.assertEqual(cfg.CMD_MIN, -cfg.CMD_MAX)
        self.assertGreater(cfg.CMD_MAX, 0)

    def test_pwm_full_scale_stays_inside_the_servo_pulse_range(self):
        self.assertGreaterEqual(cfg.PWM_NEUTRAL_US - cfg.PWM_SPAN_US, 1000)
        self.assertLessEqual(cfg.PWM_NEUTRAL_US + cfg.PWM_SPAN_US, 2000)
        self.assertGreater(cfg.PWM_SPAN_US, 0)

    def test_every_can_id_is_distinct_and_control_outranks_telemetry(self):
        ids = (cfg.CAN_ID_CONTROL,) + rp.TELEM_IDS
        self.assertEqual(len(set(ids)), len(ids), "two frames share a CAN id")
        self.assertEqual(min(ids), cfg.CAN_ID_CONTROL)
        self.assertLessEqual(max(ids), 0x7FF, "not an 11-bit standard id")


if __name__ == "__main__":
    unittest.main()
