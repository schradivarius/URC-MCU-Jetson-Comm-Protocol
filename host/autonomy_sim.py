#!/usr/bin/env python3
"""
autonomy_sim.py -- a fake autonomy stack driving the real firmware.

    python3 host/autonomy_sim.py --mock            # against the C++ simulator
    python3 host/autonomy_sim.py --port /dev/ttyACM0 # against the real Teensy

WHAT THIS IS FOR
    Task 3 proved a Jetson command can reach the controller intact. This
    proves a high-level autonomy DECISION travels that interface and makes the
    embedded layer do the low-level thing -- and, just as importantly, that the
    Jetson can read back that it did.

    It is deliberately stupid about autonomy. There is no perception, no route
    planning and no target recognition here; those belong to the real autonomy
    software and are explicitly not this module's job. What it has instead is
    the SHAPE of an autonomy stack: it decides, it commands, it reads telemetry
    back, and it refuses to believe a command worked until the controller says
    so. Swapping this for the real thing changes what decides, not how it
    talks.

THE LOOP IT CLOSES
    decision -> CONTROL frame -> validation -> mode/authority check -> drive
    module -> telemetry -> verification -> next decision

    Every step below sends through the SAME codec, the same CAN ids and the
    same frames as `jetson_test.py`. There is no side channel and no shortcut:
    if a step passes here it would pass on a real bus.

HOW A STEP IS VERIFIED, AND WHY IT IS NOT JUST A SLEEP
    Telemetry lags a command by a cycle or two, so a naive check reads the
    PREVIOUS step's state and passes for the wrong reason -- the classic way a
    green integration test proves nothing. Every step therefore records the
    sequence number of its first CONTROL frame and ignores any snapshot whose
    `ack_seq` predates it. That is what the acknowledgement field is for: the
    controller names the exact frame it acted on, so the Jetson can tell "my
    command worked" from "the last one did".
"""

import argparse
import os
import sys
import time

_PROTOCOL_ROOT = os.environ.get("ROVER_PROTOCOL_ROOT")
if _PROTOCOL_ROOT:
    sys.path.insert(0, os.path.join(
        os.path.abspath(os.path.expanduser(_PROTOCOL_ROOT)), "host"))
else:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sim_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink, open_link  # noqa: E402
from rover_config import CONTROL_RATE_HZ, SERIAL_BAUD_HZ  # noqa: E402

SEQ_LOOKAHEAD = 128

CRUISE = 400
TURN = 300
FAST = 500


def seq_at_or_after(ack_seq, first_seq):
    return rp.seq_delta(first_seq, ack_seq) < SEQ_LOOKAHEAD


class Step:
    def __init__(self, name, command, check, timeout_s=2.0):
        self.name = name
        self.command = command
        self.check = check
        self.timeout_s = timeout_s


class StepResult:
    def __init__(self, name, ok, detail, elapsed_s, snapshot):
        self.name = name
        self.ok = ok
        self.detail = detail
        self.elapsed_s = elapsed_s
        self.snapshot = snapshot

    def __str__(self):
        return "%-4s %-34s %5.2fs  %s" % ("PASS" if self.ok else "FAIL",
                                          self.name, self.elapsed_s,
                                          self.detail or "")


class MissionReport:
    def __init__(self):
        self.results = []
        self.snapshots_seen = 0

    @property
    def ok(self):
        return bool(self.results) and all(r.ok for r in self.results)

    @property
    def failed(self):
        return [r for r in self.results if not r.ok]

    def by_name(self, name):
        for r in self.results:
            if r.name == name:
                return r
        return None

    def __str__(self):
        return "\n".join(str(r) for r in self.results)


def expect(**wanted):
    def check(s):
        wrong = ["%s=%r (wanted %r)" % (k, s.get(k), v)
                 for k, v in wanted.items() if s.get(k) != v]
        return ", ".join(wrong) if wrong else None
    return check


def all_of(*checks):
    def check(s):
        for c in checks:
            problem = c(s)
            if problem is not None:
                return problem
        return None
    return check


def fault_set(bit, name):
    def check(s):
        return None if s["fault_status"] & bit else "%s is not set" % name
    return check


def fault_clear(bit, name):
    def check(s):
        return "%s is set" % name if s["fault_status"] & bit else None
    return check


def steer_feedback_within(target, tolerance):
    def check(s):
        if abs(s["steer_fb"] - target) <= tolerance:
            return None
        return "steer_fb=%d has not reached %d" % (s["steer_fb"], target)
    return check


def moving(previous_enc):
    def check(s):
        if previous_enc[0] is None:
            return "no encoder baseline was recorded by an earlier step"
        if s["enc_left"] != previous_enc[0]:
            return None
        return "enc_left is not advancing (%d)" % s["enc_left"]
    return check


def build_mission():
    enc_at_launch = [None]

    def remember_enc(s):
        enc_at_launch[0] = s["enc_left"]
        return None

    auto = dict(mode=rp.MODE_AUTONOMOUS, c2_lost=False, from_autonomy=True)
    teleop = dict(mode=rp.MODE_TELEOP, c2_lost=False, from_autonomy=False)

    grant_auto = dict(drive_cmd=0, steer_cmd=0, stop=True,
                     mode=rp.MODE_AUTONOMOUS, c2_lost=False, from_autonomy=False)
    grant_teleop = dict(drive_cmd=0, steer_cmd=0, stop=True, **teleop)


    return [
        Step("1. start in SAFE",
             dict(drive_cmd=0, steer_cmd=0, mode=rp.MODE_SAFE,
                  c2_lost=False, from_autonomy=False),
             all_of(expect(mode=rp.MODE_SAFE,
                           cmd_status=rp.CMD_STATUS_STOPPED,
                           drive_applied=0, steer_applied=0,
                           indicator_state=rp.INDICATOR_OFF),
                    fault_clear(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),

        Step("2. autonomy cannot put itself in charge",
             dict(drive_cmd=CRUISE, steer_cmd=0, **auto),
             expect(mode=rp.MODE_SAFE,
                    cmd_status=rp.CMD_STATUS_MODE_REFUSED,
                    drive_applied=0)),

        Step("3. operator grants AUTONOMOUS",
             grant_auto,
             all_of(expect(mode=rp.MODE_AUTONOMOUS,
                           cmd_status=rp.CMD_STATUS_STOPPED,
                           indicator_state=rp.INDICATOR_AUTONOMOUS),
                    fault_clear(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),

        Step("4. autonomy takes over",
             dict(drive_cmd=0, steer_cmd=0, **auto),
             all_of(expect(mode=rp.MODE_AUTONOMOUS,
                           cmd_status=rp.CMD_STATUS_APPLIED),
                    remember_enc)),

        Step("5. command forward movement",
             dict(drive_cmd=CRUISE, steer_cmd=0, **auto),
             all_of(expect(cmd_status=rp.CMD_STATUS_APPLIED,
                           drive_applied=CRUISE, steer_applied=0),
                    moving(enc_at_launch))),

        Step("6. change steering",
             dict(drive_cmd=CRUISE, steer_cmd=TURN, **auto),
             all_of(expect(cmd_status=rp.CMD_STATUS_APPLIED,
                           drive_applied=CRUISE, steer_applied=TURN),
                    steer_feedback_within(TURN, 25))),

        Step("7. stop",
             dict(drive_cmd=CRUISE, steer_cmd=TURN, stop=True, **auto),
             expect(cmd_status=rp.CMD_STATUS_STOPPED,
                    drive_applied=0, steer_applied=0,
                    indicator_state=rp.INDICATOR_AUTONOMOUS)),

        Step("8. report TARGET_REACHED",
             dict(drive_cmd=0, steer_cmd=0, stop=True,
                  indicator_request=rp.INDICATOR_ARRIVED, **auto),
             expect(indicator_state=rp.INDICATOR_ARRIVED,
                    cmd_status=rp.CMD_STATUS_STOPPED,
                    drive_applied=0)),

        Step("9. request autonomous return",
             dict(drive_cmd=0, steer_cmd=0, return_request=True,
                  indicator_request=rp.INDICATOR_ARRIVED, **auto),
             expect(return_active=True, mode=rp.MODE_AUTONOMOUS)),

        Step("10. operator takes back TELEOP",
             grant_teleop,
             expect(mode=rp.MODE_TELEOP, cmd_status=rp.CMD_STATUS_STOPPED)),

        Step("11. TELEOP refuses an autonomy command",
             dict(drive_cmd=FAST, steer_cmd=0, mode=rp.MODE_TELEOP,
                  c2_lost=False, from_autonomy=True),
             all_of(expect(mode=rp.MODE_TELEOP,
                           cmd_status=rp.CMD_STATUS_WRONG_SOURCE,
                           drive_applied=0, steer_applied=0),
                    fault_set(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),
        Step("12. operator grants AUTONOMOUS again",
             grant_auto,
             expect(mode=rp.MODE_AUTONOMOUS, cmd_status=rp.CMD_STATUS_STOPPED)),

        Step("13. AUTONOMOUS accepts the same command",
             dict(drive_cmd=FAST, steer_cmd=0, **auto),
             all_of(expect(mode=rp.MODE_AUTONOMOUS,
                           cmd_status=rp.CMD_STATUS_APPLIED,
                           drive_applied=FAST),
                    fault_clear(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),

        Step("14. AUTONOMOUS refuses a teleop command",
             dict(drive_cmd=FAST, steer_cmd=0, mode=rp.MODE_AUTONOMOUS,
                  c2_lost=False, from_autonomy=False),
             all_of(expect(cmd_status=rp.CMD_STATUS_WRONG_SOURCE,
                           drive_applied=0),
                    fault_set(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),

        Step("15. an arrival claim needs authority too",
             dict(drive_cmd=0, steer_cmd=0, mode=rp.MODE_AUTONOMOUS,
                  c2_lost=False, from_autonomy=False,
                  indicator_request=rp.INDICATOR_ARRIVED),
             expect(indicator_state=rp.INDICATOR_AUTONOMOUS)),

        Step("16. stop overrides an authorised command",
             dict(drive_cmd=FAST, steer_cmd=TURN, stop=True, **auto),
             expect(cmd_status=rp.CMD_STATUS_STOPPED, drive_applied=0)),

        Step("17. abort overrides an authorised command",
             dict(drive_cmd=FAST, steer_cmd=TURN, autonomy_abort=True, **auto),
             expect(cmd_status=rp.CMD_STATUS_STOPPED, drive_applied=0)),

        Step("18. teleop drives again afterwards",
             dict(drive_cmd=CRUISE, steer_cmd=0, **teleop),
             all_of(expect(mode=rp.MODE_TELEOP,
                           cmd_status=rp.CMD_STATUS_APPLIED,
                           drive_applied=CRUISE,
                           indicator_state=rp.INDICATOR_TELEOP),
                    fault_clear(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),
    ]


FEEDBACK_FIELDS = (
    "mode",
    "drive_applied",
    "steer_applied",
    "controller_health",
    "cmd_age_ms",
    "fault_status",
)


def run_mission(link, steps=None, verbose=True, settle_s=0.4):
    jl = JetsonLink(link)
    report = MissionReport()
    period = 1.0 / CONTROL_RATE_HZ
    steps = build_mission() if steps is None else steps

    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        jl.send_control(drive_cmd=0, steer_cmd=0, mode=rp.MODE_SAFE,
                        c2_lost=False)
        jl.poll()
        time.sleep(period)

    for step in steps:
        started = time.monotonic()
        first_seq = None
        detail = "no telemetry snapshot arrived"
        ok = False
        snapshot = None

        while time.monotonic() - started < step.timeout_s:
            sent_seq = jl.send_control(**step.command)
            if first_seq is None:
                first_seq = sent_seq
            jl.poll()

            snapshot = jl.snapshot
            if snapshot is not None:
                report.snapshots_seen = jl.tele.complete_count
                if (snapshot.get("cmd_status") == rp.CMD_STATUS_NO_COMMAND
                        or not seq_at_or_after(snapshot["ack_seq"], first_seq)):
                    detail = "still waiting for the controller to acknowledge"
                else:
                    detail = step.check(snapshot)
                    if detail is None:
                        ok = True
                        break
            time.sleep(period)

        report.results.append(StepResult(step.name, ok, detail,
                                         time.monotonic() - started, snapshot))
        if verbose:
            print(report.results[-1])
        if not ok:
            break

    return report


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", action="store_true",
                    help="run against the compiled C++ simulator (no hardware)")
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyACM0 or COM5")
    ap.add_argument("--baud", type=int, default=SERIAL_BAUD_HZ)
    args = ap.parse_args()

    if not args.mock and not args.port:
        print("Specify --mock for the simulator, or --port /dev/ttyACM0 for the Teensy.",
              file=sys.stderr)
        return 1

    print("=" * 78)
    print("Mock Jetson autonomy -> rover controller, over the Task 3 protocol")
    print("(REAL compiled firmware from firmware/src/, simulated transport)")
    print("=" * 78)

    link = open_link(args)
    try:
        report = run_mission(link)
    finally:
        link.close()

    print()
    last = report.results[-1].snapshot if report.results else None
    if last is not None:
        missing = [f for f in FEEDBACK_FIELDS if f not in last]
        print("feedback loop: %s" % ("every required field reached the Jetson"
                                     if not missing else "MISSING %s" % missing))
        print("  mode=%s  applied=(%+d,%+d)  health=%s  age=%sms  faults=%s"
              % (rp.MODE_NAMES.get(last["mode"], "?"), last["drive_applied"],
                 last["steer_applied"],
                 rp.CTRL_HEALTH_NAMES.get(last["controller_health"], "?"),
                 last["cmd_age_ms"],
                 ",".join(rp.fault_names(last["fault_status"])) or "none"))
    print("snapshots assembled: %d" % report.snapshots_seen)
    print()

    if report.ok:
        print("PASS: an autonomy decision travelled the Task 3 protocol and")
        print("reached the drive stage; the controller acknowledged each")
        print("command by sequence number; autonomy and teleop commands were")
        print("refused in each other's modes; and stop and abort overrode an")
        print("authorised autonomy command.")
        return 0
    print("FAIL at: %s" % ", ".join(r.name for r in report.failed))
    return 1


if __name__ == "__main__":
    sys.exit(main())
