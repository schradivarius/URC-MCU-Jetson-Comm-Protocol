"""
test_feedback_failures.py -- Task 5's controlled feedback failures, end to end.

Task 5 asks for "a few controlled failure conditions" so that the detection
added in v0.8 can be shown to work, and names four:

  1. encoder feedback stops updating
  2. steering feedback stops updating
  3. feedback contains an impossible value
  4. feedback is intentionally delayed

tests/cpp/test_feedback_faults.cpp already drives RoverController directly for
each of them. This file is the other half of that: the same four failures
through the REAL firmware binary (tools/rover_sim links
firmware/src/rover_controller.cpp), encoded onto the simulated bus, decoded by
the Jetson's own host/rover_protocol.py, and reassembled by the same
TelemetryAssembler the operator's dashboard uses. A detector the firmware
agrees with but the Jetson cannot read is not a detector.

The injection lives on the SIMULATED side -- firmware/src/rover_feedback_faults.h,
armed by `rover_sim --inject <name>`. It corrupts the fake sensors; the
firmware is byte-identical either way and is told nothing. Nothing on the wire
can arm it, deliberately: a way to break the rover's own sensors from the
Jetson is a weapon, not a feature.

These tests use a real subprocess and real time, so they assert on eventual
state with generous margins rather than on exact ages.
"""

import os
import subprocess
import sys
import time
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink  # noqa: E402

# Long enough for a 250 ms delay line to fill and for an age to pass the
# 150 ms staleness threshold several times over.
RUN_S = 1.2

DRIVE = 400
STEER = 300


class InjectionFixture(unittest.TestCase):
    """Drives the simulator in TELEOP and returns the last snapshot."""

    def snapshot_with(self, inject, run_s=RUN_S):
        try:
            link = can_link.SimLink(inject=inject)
        except RuntimeError as exc:
            self.skipTest(str(exc))
        try:
            jl = JetsonLink(link)
            # v0.7: only the operator may grant a motion mode, and the grant
            # is sent stopped. Same shape as RoverFixture.grant().
            self._pump(jl, 0.2, drive_cmd=0, steer_cmd=0, stop=True)
            self._pump(jl, run_s, drive_cmd=DRIVE, steer_cmd=STEER)
            snap = jl.snapshot
            self.assertIsNotNone(snap, "never received a complete snapshot")
            self.assertEqual(jl.unknown_frames, 0, "unexpected CAN id seen")
            # A fake broken SENSOR must not break the LINK: the frames still
            # arrive, still decode, and still share one seq per cycle.
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
        """Task 5: 'for now, only detect and report'. The safe state is Task 6.

        This is the assertion that would catch someone deciding on their own
        that stale feedback should stop the rover. That is a real decision with
        a real cost -- a sensor glitch stranding the rover mid-course -- so it
        belongs to whoever owns the safety argument, not to a quiet edit.
        """
        self.assertEqual(snap["mode"], rp.MODE_TELEOP, why)
        self.assertNotEqual(snap["drive_applied"], 0, why)
        self.assertEqual(snap["cmd_source"], rp.CMD_SOURCE_OPERATOR, why)


class TestHealthyBaseline(InjectionFixture):
    """Without this the four failure tests could all pass vacuously."""

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
        # Frozen, not nonsense: the repeated reading is a perfectly plausible
        # one. Only its age gives the dead channel away, which is exactly why
        # age is on the wire and not just a plausibility check.
        self.assertFalse(snap["encoder_invalid"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_ENCODER_FAULT, 0)
        # One wheel's sensor died, not the steering.
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
        # The echoed steering feedback is the frozen reading, not the honest
        # one: the injection sits at the sensor boundary, so a broken sensor
        # is wrong everywhere a real broken sensor would be. A dashboard that
        # trusted TELEM_DRIVE_L alone would see steering that never moves
        # while the command asks for %d.
        self.assertNotEqual(snap["steer_fb"], STEER)
        self.assertStillDriving(snap, "frozen steering stopped the rover")


class TestImpossibleValues(InjectionFixture):

    def test_an_impossible_encoder_value_is_reported_as_invalid(self):
        snap = self.snapshot_with("encoder-impossible")
        self.assertTrue(snap["encoder_invalid"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_ENCODER_FAULT,
                         rp.FAULT_ENCODER_FAULT)
        # Fresh and wrong. Invalid is a SEPARATE flag from stale because this
        # is what a disconnected sensor or a lost decode looks like: the
        # timestamp is current, so a staleness check alone would miss it.
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
        # Out of the command range in the echo too, and still a legal int16 on
        # the wire: the frame decodes, the VALUE is what is wrong.
        self.assertGreater(snap["steer_fb"], rp.CMD_MAX)
        self.assertStillDriving(snap, "an impossible steering value stopped the rover")


class TestDelayedFeedback(InjectionFixture):

    def test_delayed_feedback_is_reported_as_stale_not_invalid(self):
        snap = self.snapshot_with("delayed")
        self.assertTrue(snap["encoder_stale"])
        self.assertTrue(snap["steer_stale"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_STALE_FEEDBACK,
                         rp.FAULT_STALE_FEEDBACK)
        # Late, not broken: these are real readings, just old ones.
        self.assertFalse(snap["encoder_invalid"])
        self.assertFalse(snap["steer_invalid"])
        self.assertEqual(snap["fault_status"] & rp.FAULT_ENCODER_FAULT, 0)
        self.assertEqual(snap["fault_status"] & rp.FAULT_STEERING_FAULT, 0)
        self.assertStillDriving(snap, "delayed feedback stopped the rover")

    def test_a_delayed_channel_is_the_delay_asked_for(self):
        """The age settles AT the delay, which is what tells it from a freeze.

        A delay line sized in calls rather than in time reports whatever its
        ring happens to span, and on a fast loop that is far less than the
        delay asked for -- a shorter delay wearing the right name. The age has
        to land near FB_FAULT_DELAY_MS, not merely above the staleness
        threshold.
        """
        delayed = self.snapshot_with("delayed")
        frozen = self.snapshot_with("encoder-frozen")
        for key in ("encoder_age_ms", "steer_age_ms"):
            self.assertGreaterEqual(delayed[key], 200, key)
            self.assertLess(delayed[key], 400, key)
        # A frozen channel's age keeps growing instead, so after a run this
        # much longer than the delay the two cannot be confused.
        self.assertGreater(frozen["encoder_age_ms"], delayed["encoder_age_ms"] * 2)


class TestInjectionIsFailClosed(unittest.TestCase):
    """A typo must not silently run a healthy plant and report success."""

    def test_an_unknown_injection_name_is_refused(self):
        try:
            sim = can_link.build_sim()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        done = subprocess.run([sim, "--inject", "encoder-frozzen"],
                              stdin=subprocess.DEVNULL,
                              capture_output=True, text=True, timeout=30)
        self.assertEqual(done.returncode, 2, done.stderr)
        self.assertIn("unknown --inject", done.stderr)

    def test_every_injection_name_the_firmware_knows_is_accepted(self):
        """The names in the header and the ones the binary takes are one list.

        A name that parses in C++ but is rejected on the command line, or the
        other way round, means a documented failure cannot actually be run.
        """
        try:
            sim = can_link.build_sim()
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
