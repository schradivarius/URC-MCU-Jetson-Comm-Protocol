"""
test_uart_drive.py -- host/uart_drive.py against the real controller.

uart_drive.Sender only needs send(can_id, payload) and recv(), which is exactly
the interface can_link.SimLink offers over the natively compiled firmware. So
the script's own decision-making -- what it sends, when it stops, and above all
when it is allowed to say "OK" -- runs here against the real controller and
drive stage, with no pyserial and no serial port.

The acknowledgement rules get their own tests because they are where this
script once said "link is up, but nothing acknowledged" about a stop that had
worked perfectly.
"""

import contextlib
import io
import os
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
import uart_drive as ud  # noqa: E402
from rover_config import CONTROL_RATE_HZ, WATCHDOG_TIMEOUT_MS  # noqa: E402


def quiet(fn, *args):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*args)


class SilentLink:
    def __init__(self):
        self.sent = []

    def send(self, can_id, payload):
        self.sent.append((can_id, payload))

    def recv(self):
        return []


class TestAgainstTheRealController(unittest.TestCase):
    def setUp(self):
        self.link = can_link.SimLink()
        self.telemetry = ud.Telemetry()
        self.sender = ud.Sender(self.link, self.telemetry, CONTROL_RATE_HZ, True)

    def tearDown(self):
        self.link.close()

    def stop_on_exit(self):
        self.sender.run(0, 0, rp.MODE_SAFE, True,
                        ud.STOP_FRAMES / CONTROL_RATE_HZ, "stop")

    def test_a_move_is_applied_and_then_stopped(self):
        self.sender.run(300, -200, rp.MODE_TELEOP, False, 0.6, "run")
        self.assertTrue(self.telemetry.moved)
        self.assertEqual(self.telemetry.fields["drive_applied"], 300)
        self.assertEqual(self.telemetry.fields["steer_applied"], -200)

        self.stop_on_exit()
        self.assertTrue(self.telemetry.stopped)
        self.assertEqual(self.telemetry.fields["drive_applied"], 0)
        self.assertEqual(self.telemetry.fields["cmd_status"], rp.CMD_STATUS_STOPPED)
        self.assertEqual(quiet(ud.verdict, self.telemetry, True, False), 0)

    def test_a_reverse_move_is_applied(self):
        self.sender.run(-500, 0, rp.MODE_TELEOP, False, 0.6, "run")
        self.assertTrue(self.telemetry.moved)
        self.assertEqual(self.telemetry.fields["drive_applied"], -500)

    def test_a_stop_is_a_success_not_a_failure(self):
        self.sender.run(0, 0, rp.MODE_SAFE, True, 0.6, "stop")
        self.stop_on_exit()
        self.assertTrue(self.telemetry.acknowledged)
        self.assertTrue(self.telemetry.stopped)
        self.assertFalse(self.telemetry.moved)
        self.assertEqual(quiet(ud.verdict, self.telemetry, False, True), 0)

    def test_safe_mode_is_a_success_not_a_failure(self):
        self.sender.run(0, 0, rp.MODE_SAFE, False, 0.6, "run")
        self.stop_on_exit()
        self.assertEqual(quiet(ud.verdict, self.telemetry, False, True), 0)

    def test_a_zero_teleop_command_is_acknowledged(self):
        self.sender.run(0, 0, rp.MODE_TELEOP, False, 0.6, "run")
        self.stop_on_exit()
        self.assertTrue(self.telemetry.acknowledged)
        self.assertEqual(quiet(ud.verdict, self.telemetry, False, False), 0)

    def test_the_stop_on_exit_continues_the_same_sequence(self):
        self.sender.run(200, 0, rp.MODE_TELEOP, False, 0.4, "run")
        first = self.sender.seq
        self.stop_on_exit()
        self.assertGreater(self.sender.seq, first)
        self.assertEqual(self.telemetry.sent_seqs, set(range(self.sender.sent)))
        self.assertEqual(self.telemetry.fields["jetson_link"], rp.LINK_OK,
                         "the stop phase restarted the sequence numbers, and "
                         "the firmware reported it as a gap")

    def test_the_link_stays_clean_for_a_whole_run(self):
        self.sender.run(300, 0, rp.MODE_TELEOP, False, 0.8, "run")
        self.stop_on_exit()
        self.assertEqual(self.telemetry.decode_errors, 0)
        self.assertEqual(rp.fault_names(self.telemetry.fields["fault_status"]), [])


class TestAcknowledgementRules(unittest.TestCase):
    def feed_cycle(self, telemetry, ack_seq, cmd_age_ms, status, drive=0):
        telemetry.ingest(rp.CAN_ID_TELEM_DRIVE_R,
                         rp.encode_telemetry_drive_r(0, cmd_age_ms, seq=1))
        telemetry.ingest(rp.CAN_ID_TELEM_CMD_ECHO,
                         rp.encode_telemetry_cmd_echo(
                             drive, 0, ack_seq, rp.pack_cmd_status(status), seq=1))

    def test_an_echo_of_a_frame_this_run_never_sent_does_not_count(self):
        telemetry = ud.Telemetry()
        telemetry.sent_seqs.update(range(10))
        self.feed_cycle(telemetry, 200, 20, rp.CMD_STATUS_APPLIED, drive=300)
        self.assertFalse(telemetry.acknowledged)
        self.assertFalse(telemetry.moved)

    def test_a_stale_echo_does_not_count_even_if_the_seq_matches(self):
        telemetry = ud.Telemetry()
        telemetry.sent_seqs.update(range(10))
        self.feed_cycle(telemetry, 5, WATCHDOG_TIMEOUT_MS, rp.CMD_STATUS_APPLIED,
                        drive=300)
        self.assertFalse(telemetry.acknowledged)

    def test_an_echo_before_any_command_does_not_count(self):
        telemetry = ud.Telemetry()
        telemetry.sent_seqs.add(0)
        self.feed_cycle(telemetry, 0, rp.CMD_AGE_UNKNOWN, rp.CMD_STATUS_NO_COMMAND)
        self.assertFalse(telemetry.acknowledged)

    def test_a_fresh_echo_of_this_run_counts(self):
        telemetry = ud.Telemetry()
        telemetry.sent_seqs.update(range(10))
        self.feed_cycle(telemetry, 7, 40, rp.CMD_STATUS_APPLIED, drive=300)
        self.assertTrue(telemetry.acknowledged)
        self.assertTrue(telemetry.moved)

    def test_applied_at_zero_is_not_motion(self):
        telemetry = ud.Telemetry()
        telemetry.sent_seqs.add(3)
        self.feed_cycle(telemetry, 3, 40, rp.CMD_STATUS_APPLIED, drive=0)
        self.assertTrue(telemetry.acknowledged)
        self.assertFalse(telemetry.moved)


class TestVerdict(unittest.TestCase):
    def test_silence_is_exit_one(self):
        link = SilentLink()
        telemetry = ud.Telemetry()
        sender = ud.Sender(link, telemetry, CONTROL_RATE_HZ, True)
        sender.run(300, 0, rp.MODE_TELEOP, False, 0.2, "run")
        self.assertGreater(len(link.sent), 0)
        self.assertEqual(quiet(ud.verdict, telemetry, True, False), 1)

    def test_motion_without_a_confirmed_stop_is_a_failure(self):
        telemetry = ud.Telemetry()
        telemetry.frames = 5
        telemetry.acknowledged = True
        telemetry.moved = True
        telemetry.stopped = False
        self.assertEqual(quiet(ud.verdict, telemetry, True, False), 2)

    def test_telemetry_without_an_acknowledgement_is_exit_two(self):
        telemetry = ud.Telemetry()
        telemetry.frames = 5
        self.assertEqual(quiet(ud.verdict, telemetry, False, True), 2)


class TestArguments(unittest.TestCase):
    def run_main(self, *argv):
        old = sys.argv
        sys.argv = ["uart_drive.py"] + list(argv)
        try:
            with contextlib.redirect_stderr(io.StringIO()), \
                 contextlib.redirect_stdout(io.StringIO()):
                return ud.main()
        except SystemExit as exc:
            return exc.code
        finally:
            sys.argv = old

    def test_autonomous_is_not_offered(self):
        self.assertEqual(self.run_main("--port", "X", "--mode", "autonomous"), 2)

    def test_an_out_of_range_drive_is_refused_before_anything_is_sent(self):
        self.assertEqual(self.run_main("--port", "X", "--drive", "1001"), 2)

    def test_stop_and_drive_together_are_refused(self):
        self.assertEqual(self.run_main("--port", "X", "--stop", "--drive", "100"), 2)

    def test_safe_mode_cannot_move(self):
        self.assertEqual(self.run_main("--port", "X", "--mode", "safe",
                                       "--drive", "100"), 2)

    def test_a_rate_too_slow_for_the_watchdog_is_refused(self):
        self.assertEqual(self.run_main("--port", "X", "--rate", "5"), 2)


if __name__ == "__main__":
    unittest.main()
