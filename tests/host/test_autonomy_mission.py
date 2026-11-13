import os
import sys
import time
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_OVERRIDE = os.environ.get("ROVER_PROTOCOL_ROOT")
if _OVERRIDE:
    sys.path.insert(0, os.path.join(
        os.path.abspath(os.path.expanduser(_OVERRIDE)), "host"))
sys.path.insert(0, os.path.join(REPO, "host"))

import autonomy_sim as mission  # noqa: E402
import sim_link  # noqa: E402
import rover_protocol as rp  # noqa: E402


class MissionFixture(unittest.TestCase):
    def setUp(self):
        try:
            self.link = sim_link.SimLink()
        except RuntimeError as exc:
            self.skipTest(str(exc))

    def tearDown(self):
        self.link.close()


class TestTheMissionRuns(MissionFixture):
    def test_the_whole_mission_passes(self):
        report = mission.run_mission(self.link, verbose=False)
        self.assertTrue(
            report.ok,
            "autonomy mission failed at: %s"
            % "; ".join("%s (%s)" % (r.name, r.detail) for r in report.failed))
        self.assertEqual(len(report.results), len(mission.build_mission()),
                         "the run stopped early without reporting a failure")

    def test_every_step_passed_on_telemetry_it_caused(self):
        report = mission.run_mission(self.link, verbose=False)
        self.assertTrue(report.ok)
        acks = [r.snapshot["ack_seq"] for r in report.results]
        for earlier, later in zip(acks, acks[1:]):
            self.assertNotEqual(
                earlier, later,
                "two steps passed on the same acknowledged frame, so at least "
                "one of them was reading stale telemetry")

    def test_the_feedback_loop_carries_every_required_field(self):
        report = mission.run_mission(self.link, verbose=False)
        self.assertTrue(report.ok)
        snapshot = report.results[-1].snapshot
        for field in mission.FEEDBACK_FIELDS:
            self.assertIn(field, snapshot, "the feedback loop is missing %r" % field)

    def test_the_mode_separation_steps_disagree_about_the_same_command(self):
        report = mission.run_mission(self.link, verbose=False)
        self.assertTrue(report.ok)
        refused = report.by_name("11. TELEOP refuses an autonomy command")
        accepted = report.by_name("13. AUTONOMOUS accepts the same command")
        self.assertIsNotNone(refused)
        self.assertIsNotNone(accepted)
        self.assertEqual(refused.snapshot["drive_applied"], 0)
        self.assertEqual(accepted.snapshot["drive_applied"], mission.FAST)
        self.assertNotEqual(refused.snapshot["cmd_status"],
                            accepted.snapshot["cmd_status"])


class TestTheMissionCanFail(MissionFixture):
    def test_an_impossible_expectation_fails_the_run(self):
        steps = [
            mission.Step("ok", dict(drive_cmd=0, steer_cmd=0,
                                    mode=rp.MODE_SAFE, c2_lost=False),
                         mission.expect(mode=rp.MODE_SAFE), timeout_s=1.5),
            mission.Step("impossible", dict(drive_cmd=0, steer_cmd=0,
                                            mode=rp.MODE_SAFE, c2_lost=False),
                         mission.expect(mode=rp.MODE_AUTONOMOUS), timeout_s=0.7),
            mission.Step("never reached", dict(drive_cmd=0, steer_cmd=0,
                                               mode=rp.MODE_SAFE, c2_lost=False),
                         mission.expect(mode=rp.MODE_SAFE), timeout_s=0.7),
        ]
        report = mission.run_mission(self.link, steps=steps, verbose=False)
        self.assertFalse(report.ok)
        self.assertEqual([r.name for r in report.failed], ["impossible"])
        self.assertEqual(len(report.results), 2)
        self.assertIn("mode=", report.failed[0].detail)

    def test_a_step_that_never_gets_telemetry_fails_rather_than_hanging(self):
        self.link.close()
        steps = [mission.Step("no controller",
                              dict(drive_cmd=0, steer_cmd=0,
                                   mode=rp.MODE_SAFE, c2_lost=False),
                              mission.expect(mode=rp.MODE_SAFE),
                              timeout_s=0.7)]
        started = time.monotonic()
        report = mission.run_mission(self.link, steps=steps, verbose=False,
                                     settle_s=0.1)
        self.assertFalse(report.ok)
        self.assertLess(time.monotonic() - started, 5.0,
                        "a dead controller hung the mission instead of failing")


class TestTheSequenceGate(unittest.TestCase):
    def test_a_later_sequence_number_counts_as_fresh(self):
        self.assertTrue(mission.seq_at_or_after(10, 10))
        self.assertTrue(mission.seq_at_or_after(11, 10))
        self.assertTrue(mission.seq_at_or_after(100, 10))

    def test_an_earlier_sequence_number_does_not(self):
        self.assertFalse(mission.seq_at_or_after(9, 10))
        self.assertFalse(mission.seq_at_or_after(0, 10))

    def test_it_is_correct_across_the_wrap(self):
        self.assertTrue(mission.seq_at_or_after(1, 254))
        self.assertTrue(mission.seq_at_or_after(0, 255))
        self.assertFalse(mission.seq_at_or_after(253, 254))
        self.assertFalse(mission.seq_at_or_after(254, 255))

    def test_the_lookahead_splits_the_sequence_space_evenly(self):
        self.assertEqual(mission.SEQ_LOOKAHEAD * 2, 256)


if __name__ == "__main__":
    unittest.main()
