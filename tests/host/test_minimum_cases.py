"""
test_minimum_cases.py -- the eight minimum test cases from the Task 3 brief,
one test per case, in the brief's own order and vocabulary.

WHY THIS FILE EXISTS SEPARATELY FROM test_integration.py
    test_integration.py is organised by mechanism: snapshot assembly, link
    separation, bus hygiene. That is the right shape for finding a regression,
    and the wrong shape for answering "did we run case 6?". This file is the
    traceable answer to that question: eight tests, numbered as the brief
    numbers them, each one running the brief's scenario end to end against the
    real compiled controller. Overlap with test_integration.py is deliberate --
    the two files are indexed differently on purpose.

EVERY CASE RUNS THROUGH tools/rover_sim, which links the REAL
firmware/src/rover_controller.cpp. Nothing here tests a Python mock.
"""

import os
import sys
import time
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink  # noqa: E402


class Case(unittest.TestCase):
    """Shared harness: a Jetson talking to the real controller over a sim bus."""

    def setUp(self):
        try:
            self.link = can_link.SimLink()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        self.jl = JetsonLink(self.link)

    def tearDown(self):
        self.link.close()

    def drive(self, seconds, send=True, **kw):
        """Send CONTROL at the real 20 Hz rate, or stay silent, for `seconds`."""
        kw.setdefault("drive_cmd", 400)
        kw.setdefault("steer_cmd", 100)
        kw.setdefault("mode", rp.MODE_MANUAL)
        kw.setdefault("c2_lost", False)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if send:
                self.jl.send_control(**kw)
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()

    def send_raw(self, frame, seconds):
        """Put arbitrary bytes on the CONTROL id, as a faulty sender would."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.link.send(rp.CAN_ID_CONTROL, frame)
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()

    def snap(self):
        self.assertIsNotNone(self.jl.snapshot,
                             "no complete telemetry snapshot was assembled")
        return self.jl.snapshot

    def faults(self):
        return rp.fault_names(self.snap()["fault_status"])


class Case1NormalTeleopPacket(Case):
    """1. mode = TELEOP, valid drive/steering, CRC passes, MCU accepts and
    returns telemetry."""

    def test_case_1_normal_teleop_packet(self):
        self.drive(0.6, drive_cmd=400, steer_cmd=100, mode=rp.MODE_MANUAL,
                   indicator_request=rp.INDICATOR_TELEOP)
        s = self.snap()

        # CRC passed and the frame was accepted: no protocol or CRC fault, and
        # the command watchdog is being fed.
        self.assertEqual(self.faults(), [], "a valid teleop packet raised a fault")
        self.assertEqual(s["jetson_link"], rp.LINK_OK)
        self.assertLess(s["cmd_age_ms"], 300, "command age is already stale")

        # The MCU acted on the command.
        self.assertGreater(s["enc_left"], 0, "the rover never moved")
        self.assertGreater(s["current_a"], 0, "no current drawn while driving")

        # And returned telemetry: every field of the snapshot is present and
        # the mode echo agrees with what was sent.
        self.assertEqual(s["mode"], rp.MODE_MANUAL)
        self.assertEqual(s["indicator_state"], rp.INDICATOR_TELEOP)
        self.assertGreater(self.jl.tele.complete_count, 5,
                           "telemetry did not come back at the expected rate")


class Case2NormalAutonomousPacket(Case):
    """2. mode = AUTONOMOUS, valid motion command, red status requested, MCU
    accepts state."""

    def test_case_2_normal_autonomous_packet(self):
        self.drive(0.6, drive_cmd=600, steer_cmd=-200,
                   mode=rp.MODE_AUTONOMOUS,
                   indicator_request=rp.INDICATOR_AUTONOMOUS)
        s = self.snap()

        self.assertEqual(self.faults(), [],
                         "a valid autonomous packet raised a fault")
        self.assertEqual(s["mode"], rp.MODE_AUTONOMOUS,
                         "the MCU did not accept the autonomous mode")

        # The motion command was acted on, not merely accepted.
        self.assertGreater(s["enc_left"], 0, "the rover never moved")
        self.assertGreater(s["current_a"], 0)

        # Red is what URC requires during autonomous operation, and it is what
        # the Jetson asked for, so the two agree.
        self.assertEqual(s["indicator_state"], rp.INDICATOR_AUTONOMOUS,
                         "an autonomous rover was not showing red")


class Case3TargetReached(Case):
    """3. arrival/status flag sent, MCU reports flashing-green status state."""

    def test_case_3_target_reached(self):
        # The realistic sequence, and the one URC scores: the rover drives
        # autonomously, then STOPS at the target, and only then indicates
        # arrival. Rule 1.e.xvii awards the points for "autonomously stopping
        # within 1m of the target location AND indicating its arrival", so
        # testing the flag while the rover is still driving would test a state
        # that never earns anything.
        self.drive(0.5, drive_cmd=600, mode=rp.MODE_AUTONOMOUS,
                   indicator_request=rp.INDICATOR_AUTONOMOUS)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_AUTONOMOUS)
        travelled = self.snap()["enc_left"]
        self.assertGreater(travelled, 0, "the rover never approached a target")

        # Arrived: the Jetson commands a halt and raises the arrival flag.
        self.drive(0.6, drive_cmd=0, steer_cmd=0, mode=rp.MODE_AUTONOMOUS,
                   stop=True, indicator_request=rp.INDICATOR_ARRIVED)
        s = self.snap()
        self.assertEqual(s["indicator_state"], rp.INDICATOR_ARRIVED,
                         "a rover stopped at its target did not signal arrival")

        # Stopped, and still in autonomous operation while it signals.
        self.assertEqual(s["current_ca"], 0, "still drawing current at the target")
        self.assertEqual(s["mode"], rp.MODE_AUTONOMOUS)

        # It must hold the signal, not blink it for one cycle.
        held = self.snap()["enc_left"]
        self.drive(0.5, drive_cmd=0, steer_cmd=0, mode=rp.MODE_AUTONOMOUS,
                   stop=True, indicator_request=rp.INDICATOR_ARRIVED)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_ARRIVED,
                         "the arrival signal did not persist")
        self.assertEqual(self.snap()["enc_left"], held,
                         "the rover crept while signalling arrival")


class Case4CorruptedPacket(Case):
    """4. intentionally bad CRC, MCU rejects it."""

    def test_case_4_corrupted_packet(self):
        # Establish how far this rover travels in 0.4 s under a VALID drive of
        # 400, so the corrupted frame can be judged against a real yardstick
        # rather than against zero. A rejected frame does not teleport the
        # rover to a halt: the last valid command stays in force until it ages
        # out, which is correct and is why "it moved at all" proves nothing.
        self.drive(0.4, drive_cmd=400)
        mark = self.snap()["enc_left"]
        self.drive(0.4, drive_cmd=400)
        under_valid_command = self.snap()["enc_left"] - mark
        self.assertGreater(under_valid_command, 0)
        before = self.snap()["enc_left"]

        # Corrupt the payload AFTER the CRC was computed over it. This is the
        # software-path corruption CAN's own CRC cannot see, which is the whole
        # reason the protocol carries an application-layer CRC of its own.
        # The corrupted frame asks for 900 -- more than double what is in
        # force -- for a longer window than the yardstick above.
        good = rp.encode_control(900, 500, rp.MODE_MANUAL, False, seq=50)
        bad = bytearray(good)
        bad[0] ^= 0xFF
        self.send_raw(bytes(bad), 0.7)

        self.assertIn("CRC_ERROR", self.faults(), "the bad CRC was not reported")
        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "a CRC-invalid frame refreshed the command watchdog")
        self.assertEqual(self.snap()["current_ca"], 0,
                         "the rejected frame's command was acted on")

        # Had the frame been accepted, the rover would have run at 900 for the
        # whole 0.7 s. Rejected, it coasts on the old 400 until the watchdog
        # trips at 300 ms and then freezes -- strictly less ground than the
        # valid-command yardstick covered in 0.4 s.
        crept = self.snap()["enc_left"] - before
        self.assertLess(crept, under_valid_command,
                        "the rover acted on a frame it had rejected")

        # And it stays frozen: the stale command is not resurrected.
        frozen = self.snap()["enc_left"]
        self.send_raw(bytes(bad), 0.4)
        self.assertEqual(self.snap()["enc_left"], frozen,
                         "the rover moved again on further rejected frames")


class Case5StaleCommand(Case):
    """5. stop sending commands, command age exceeds timeout, MCU invalidates
    the stale command."""

    def test_case_5_stale_command(self):
        self.drive(0.6, drive_cmd=600)
        fresh = self.snap()
        self.assertLess(fresh["cmd_age_ms"], 300)
        moving = fresh["enc_left"]

        # Silence. The last command said "drive at 600" and is still the most
        # recent thing the controller was told; age is the only thing that
        # invalidates it.
        self.drive(0.7, send=False)
        stale = self.snap()

        # The age itself is the subject of this case, so assert on it directly
        # rather than inferring it from the fault bit.
        self.assertGreater(stale["cmd_age_ms"], 300,
                           "command age did not climb past the timeout")
        self.assertNotEqual(stale["cmd_age_ms"], rp.CMD_AGE_UNKNOWN,
                            "a command that did arrive is reported as never-seen")
        self.assertIn("COMM_TIMEOUT", self.faults())

        # Invalidated: the stale drive command is no longer acted on.
        self.assertEqual(stale["current_ca"], 0)
        frozen = stale["enc_left"]
        self.assertGreater(frozen, moving)
        self.drive(0.4, send=False)
        self.assertEqual(self.snap()["enc_left"], frozen,
                         "the rover kept acting on a command past its timeout")
        self.assertGreater(self.snap()["cmd_age_ms"], stale["cmd_age_ms"],
                           "command age stopped climbing while still silent")


class Case6JetsonHeartbeatLost(Case):
    """6. Jetson stops transmitting, MCU reports Jetson-loss state."""

    def test_case_6_jetson_heartbeat_lost(self):
        self.drive(0.5)
        self.assertEqual(self.snap()["jetson_link"], rp.LINK_OK)

        self.drive(0.7, send=False)
        s = self.snap()

        # Reported three ways, all of which an operator might be looking at.
        self.assertEqual(s["jetson_link"], rp.LINK_LOST,
                         "the Jetson link was not reported lost")
        self.assertIn("COMM_TIMEOUT", self.faults())
        self.assertEqual(s["indicator_state"], rp.INDICATOR_FAULT,
                         "a rover nothing is driving showed a healthy light")

        # Telemetry keeps flowing while the link is down -- that is how the
        # loss is visible at all.
        before = self.jl.tele.complete_count
        self.drive(0.3, send=False)
        self.assertGreater(self.jl.tele.complete_count, before,
                           "telemetry stopped when the Jetson went silent")

        # And it recovers by itself when the Jetson comes back.
        self.drive(0.5)
        self.assertEqual(self.snap()["jetson_link"], rp.LINK_OK)
        self.assertEqual(self.faults(), [])


class Case7C2LostJetsonAlive(Case):
    """7. simulate C2-loss flag while the Jetson heartbeat continues; the MCU
    must distinguish this from Jetson failure."""

    def test_case_7_c2_lost_but_jetson_still_alive(self):
        # The 2027 out-of-line-of-sight case: the rover drives behind the hill,
        # the base station link drops, onboard autonomy carries on.
        self.drive(0.6, drive_cmd=700, mode=rp.MODE_AUTONOMOUS, c2_lost=True)
        s = self.snap()

        # Distinguished: two separate link fields, saying two different things
        # at the same moment.
        self.assertEqual(s["c2_link"], rp.LINK_LOST, "C2 loss was not reported")
        self.assertEqual(s["jetson_link"], rp.LINK_OK,
                         "a C2 loss was misreported as a Jetson failure")
        self.assertTrue(s["fault_status"] & rp.FAULT_C2_LINK_LOST)
        self.assertFalse(s["fault_status"] & rp.FAULT_COMM_TIMEOUT,
                         "C2 loss raised the Jetson heartbeat fault")

        # And NOT equivalent in behaviour: autonomy keeps driving. This is the
        # 25-point difference on the autonomy course.
        self.assertGreater(s["enc_left"], 0,
                           "an autonomous rover was stopped by C2 loss alone")
        self.assertNotEqual(s["current_a"], 0.0)

        # The same flag in MANUAL does stop the rover, because there the
        # operator's commands are what can no longer arrive.
        self.drive(0.6, drive_cmd=700, mode=rp.MODE_MANUAL, c2_lost=True)
        self.assertEqual(self.snap()["current_a"], 0.0,
                         "a teleoperated rover kept driving with C2 lost")
        self.assertEqual(self.snap()["jetson_link"], rp.LINK_OK,
                         "the Jetson link was blamed for a C2 loss")


class Case8AbortReturnRequest(Case):
    """8. send abort or return command; the MCU receives and acknowledges the
    correct command type."""

    def test_case_8_abort_is_received_and_acknowledged(self):
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS)
        self.assertGreater(self.snap()["enc_left"], 0)

        self.drive(0.6, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   autonomy_abort=True)
        s = self.snap()

        # Acknowledged: the Jetson can read back which command the controller
        # is holding, and it is the abort, not the return.
        self.assertTrue(s["command_ack"] & rp.ACK_AUTONOMY_ABORT,
                        "the abort was never acknowledged in telemetry")
        self.assertFalse(s["command_ack"] & rp.ACK_RETURN_REQUEST,
                         "an abort was acknowledged as a return request")

        # Received, in the sense that matters: it acted on it.
        halted = s["enc_left"]
        self.drive(0.4, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   autonomy_abort=True)
        self.assertEqual(self.snap()["enc_left"], halted,
                         "kept driving autonomously after an abort")

    def test_case_8_return_is_received_and_acknowledged(self):
        self.drive(0.6, drive_cmd=600, mode=rp.MODE_AUTONOMOUS,
                   return_request=True)
        s = self.snap()

        self.assertTrue(s["command_ack"] & rp.ACK_RETURN_REQUEST,
                        "the return request was never acknowledged in telemetry")
        self.assertFalse(s["command_ack"] & rp.ACK_AUTONOMY_ABORT,
                         "a return request was acknowledged as an abort")

        # A return request is a message for the autonomy layer, not a brake.
        # The MCU carries it; the Jetson plans the route.
        self.assertGreater(s["enc_left"], 0,
                           "a return request stopped the rover")

        # And it must not be confused with arrival: the rover has been told to
        # come back, not that it got somewhere. Flashing green is scored.
        self.assertNotEqual(s["indicator_state"], rp.INDICATOR_ARRIVED,
                            "a return request was signalled as an arrival")

    def test_case_8_the_two_requests_are_distinguishable_together(self):
        # Both at once: abort the current task AND come home. The acknowledgment
        # has to carry both, or the operator cannot tell which was honoured.
        self.drive(0.6, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   autonomy_abort=True, return_request=True)
        ack = self.snap()["command_ack"]
        self.assertTrue(ack & rp.ACK_AUTONOMY_ABORT)
        self.assertTrue(ack & rp.ACK_RETURN_REQUEST)

        # And they clear when the Jetson stops asking, so the acknowledgment
        # reports a live state rather than latching for the session.
        self.drive(0.5, drive_cmd=400, mode=rp.MODE_AUTONOMOUS)
        ack = self.snap()["command_ack"]
        self.assertFalse(ack & rp.ACK_AUTONOMY_ABORT,
                         "the abort acknowledgment latched after it was released")
        self.assertFalse(ack & rp.ACK_RETURN_REQUEST,
                         "the return acknowledgment latched after it was released")


if __name__ == "__main__":
    unittest.main()
