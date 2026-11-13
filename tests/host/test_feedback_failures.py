import os
import subprocess
import sys
import time
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import sim_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink  # noqa: E402

RUN_S = 1.2

DRIVE = 400
STEER = 300


class InjectionFixture(unittest.TestCase):
    def snapshot_with(self, inject, run_s=RUN_S):
        try:
            link = sim_link.SimLink(inject=inject)
        except RuntimeError as exc:
            self.skipTest(str(exc))
        try:
            jl = JetsonLink(link)
            self._pump(jl, 0.2, drive_cmd=0, steer_cmd=0, stop=True)
            self._pump(jl, run_s, drive_cmd=DRIVE, steer_cmd=STEER)
            snap = jl.snapshot
            self.assertIsNotNone(snap, "never received a complete snapshot")
            self.assertEqual(jl.unknown_frames, 0, "unexpected CAN id seen")
            self.assertEqual(jl.tele.decode_errors, {},
                             "telemetry failed to decode: %r" % jl.tele.decode_errors)
            self.assertGreater(jl.tele.complete_count, 10, "too few snapshots")
            return snap
        finally:
            link.close()

    @staticmethod
    def _pump(jl, seconds, **kw):
        kw.setdefault("mode", rp.MODE_TELEOP)
        kw.setdefault("c2_lost", False)
        kw.setdefault("from_autonomy", False)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            jl.send_control(**kw)
            jl.poll()
            time.sleep(0.02)
        jl.poll()

    def assertStillDriving(self, snap, why):
        self.assertEqual(snap["mode"], rp.MODE_TELEOP, why)
        self.assertNotEqual(snap["drive_applied"], 0, why)
        self.assertEqual(snap["cmd_source"], rp.CMD_SOURCE_OPERATOR, why)


class TestHealthyBaseline(InjectionFixture):
    def test_a_healthy_plant_reports_nothing(self):
        snap = self.snapshot_with(None)
        self.assertEqual(snap["feedback_flags"], 0,
                         "a healthy plant raised %s"
                         % rp.feedback_flag_names(snap["feedback_flags"]))
        self.assertEqual(snap["fault_status"] & rp.FAULT_STALE_FEEDBACK, 0)
        self.assertEqual(snap["fault_status"] & rp.FAULT_ENCODER_FAULT, 0)
        self.assertEqual(snap["fault_status"] & rp.FAULT_STEERING_FAULT, 0)
        self.assertLess(snap["encoder_age_ms"], rp.FEEDBACK_STALE_MS)
        self.assertLess(snap["steer_age_ms"], rp.FEEDBACK_STALE_MS)
        self.assertStillDriving(snap, "a healthy plant stopped the rover")


class TestEncoderStopsUpdating(InjectionFixture):
    def test_a_frozen_encoder_reaches_the_jetson_as_stale(self):
        snap = self.snapshot_with("encoder-frozen")
        self.assertTrue(snap["encoder_stale"])
        self.assertGreaterEqual(snap["encoder_age_ms"], rp.FEEDBACK_STALE_MS)
        self.assertEqual(snap["fault_status"] & rp.FAULT_STALE_FEEDBACK,
                         rp.FAULT_STALE_FEEDBACK)
        self.assertFalse(snap["encoder_invalid"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_ENCODER_FAULT, 0)
        self.assertFalse(snap["steer_stale"])
        self.assertLess(snap["steer_age_ms"], rp.FEEDBACK_STALE_MS)
        self.assertStillDriving(snap, "a frozen encoder stopped the rover")


class TestSteeringStopsUpdating(InjectionFixture):
    def test_frozen_steering_reaches_the_jetson_as_stale(self):
        snap = self.snapshot_with("steer-frozen")
        self.assertTrue(snap["steer_stale"])
        self.assertGreaterEqual(snap["steer_age_ms"], rp.FEEDBACK_STALE_MS)
        self.assertEqual(snap["fault_status"] & rp.FAULT_STALE_FEEDBACK,
                         rp.FAULT_STALE_FEEDBACK)
        self.assertFalse(snap["steer_invalid"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_STEERING_FAULT, 0)
        self.assertFalse(snap["encoder_stale"])
        self.assertNotEqual(snap["steer_fb"], STEER)
        self.assertStillDriving(snap, "frozen steering stopped the rover")


class TestImpossibleValues(InjectionFixture):
    def test_an_impossible_encoder_value_is_reported_as_invalid(self):
        snap = self.snapshot_with("encoder-impossible")
        self.assertTrue(snap["encoder_invalid"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_ENCODER_FAULT,
                         rp.FAULT_ENCODER_FAULT)
        self.assertFalse(snap["encoder_stale"])
        self.assertLess(snap["encoder_age_ms"], rp.FEEDBACK_STALE_MS)
        self.assertEqual(snap["fault_status"] & rp.FAULT_STALE_FEEDBACK, 0)
        self.assertStillDriving(snap, "an impossible encoder value stopped the rover")

    def test_an_impossible_steering_value_is_reported_as_invalid(self):
        snap = self.snapshot_with("steer-impossible")
        self.assertTrue(snap["steer_invalid"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_STEERING_FAULT,
                         rp.FAULT_STEERING_FAULT)
        self.assertFalse(snap["steer_stale"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_STALE_FEEDBACK, 0)
        self.assertGreater(snap["steer_fb"], rp.CMD_MAX)
        self.assertStillDriving(snap, "an impossible steering value stopped the rover")


class TestDelayedFeedback(InjectionFixture):
    def test_delayed_feedback_is_reported_as_stale_not_invalid(self):
        snap = self.snapshot_with("delayed")
        self.assertTrue(snap["encoder_stale"])
        self.assertTrue(snap["steer_stale"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_STALE_FEEDBACK,
                         rp.FAULT_STALE_FEEDBACK)
        self.assertFalse(snap["encoder_invalid"])
        self.assertFalse(snap["steer_invalid"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_ENCODER_FAULT, 0)
        self.assertEqual(snap["fault_status"] & rp.FAULT_STEERING_FAULT, 0)
        self.assertStillDriving(snap, "delayed feedback stopped the rover")

    def test_a_delayed_channel_is_the_delay_asked_for(self):
        delayed = self.snapshot_with("delayed")
        frozen = self.snapshot_with("encoder-frozen")
        for key in ("encoder_age_ms", "steer_age_ms"):
            self.assertGreaterEqual(delayed[key], 200, key)
            self.assertLess(delayed[key], 400, key)
        self.assertGreater(frozen["encoder_age_ms"], delayed["encoder_age_ms"] * 2)


class TestInjectionIsFailClosed(unittest.TestCase):
    def test_an_unknown_injection_name_is_refused(self):
        try:
            sim = sim_link.build_sim()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        done = subprocess.run([sim, "--inject", "encoder-frozzen"],
                              stdin=subprocess.DEVNULL,
                              capture_output=True, text=True, timeout=30)
        self.assertEqual(done.returncode, 2, done.stderr)
        self.assertIn("unknown --inject", done.stderr)

    def test_every_injection_name_the_firmware_knows_is_accepted(self):
        try:
            sim = sim_link.build_sim()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        header = os.path.join(REPO, "firmware", "src", "rover_feedback_faults.cpp")
        with open(header) as fh:
            body = fh.read()
        names = [line.split('"')[1] for line in body.splitlines()
                 if line.strip().startswith("{FB_FAULT_")]
        self.assertIn("delayed", names)
        self.assertEqual(len(names), 6, names)
        for name in names:
            with self.subTest(name=name):
                proc = subprocess.Popen([sim, "--inject", name],
                                        stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True)
                try:
                    time.sleep(0.2)
                    self.assertIsNone(proc.poll(),
                                      "rover_sim exited on --inject %s" % name)
                finally:
                    proc.stdin.close()
                    proc.terminate()
                    proc.wait(timeout=10)
                    proc.stdout.close()


if __name__ == "__main__":
    unittest.main()
