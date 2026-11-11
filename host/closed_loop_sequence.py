"""
closed_loop_sequence.py

Modified autonomy_sim.py that executes a full closed loop sequence instead of the task 3 control and autonomy state validation

    python3 host/closed_loop_sequence.py --mock            # against the C++ simulator
    python3 host/closed_loop_sequence.py --port /dev/ttyACM0 # against the real Teensy

    Every snapshot the mission reads is written to closed_loop_run.csv
    (change it with --record FILE).

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
import csv
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
from rover_config import (  # noqa: E402
    CONTROL_RATE_HZ, SERIAL_BAUD_HZ, TELEMETRY_PERIOD_MS)

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


def feedback_is_fresh():
    def check(s):
        problems = []
        for field in ("encoder_age_ms", "steer_age_ms"):
            if s[field] == rp.FEEDBACK_AGE_UNKNOWN:
                problems.append("%s never reported" % field)
            elif s[field] >= rp.FEEDBACK_STALE_MS:
                problems.append("%s=%dms is stale" % (field, s[field]))
        if s["feedback_flags"]:
            problems.append("flags=%s"
                            % ",".join(rp.feedback_flag_names(s["feedback_flags"])))
        return ", ".join(problems) if problems else None
    return check


def steer_feedback_within(target, tolerance):
    def check(s):
        if abs(s["steer_fb"] - target) <= tolerance:
            return None
        return "steer_fb=%d has not reached %d" % (s["steer_fb"], target)
    return check


SPEED_TOLERANCE = 0.10
STOPPED_MAX = 200
SETTLE_READINGS = 4

TELEMETRY_PERIOD_S = TELEMETRY_PERIOD_MS / 1000.0


def speed_between(prev, curr):
    if prev is None:
        return None
    count1, t1 = prev
    count2, t2 = curr
    count = count2 - count1
    dt = t2 - t1
    if dt <= 0:
        return None
    speed = count / dt
    return speed


def speed_problem(measured, target, pct=SPEED_TOLERANCE, stopped_max=STOPPED_MAX):
    if measured is None:
        return "Nothing measured yet."
    if target is None:
        return "Target not set"
    if target == 0:
        if abs(measured) <= stopped_max:
            return None
        else:
            return f"speed {measured:.0f} is not under {stopped_max} (should be stopped)"

    error = measured - target
    allowed = pct * abs(target)
    if abs(error) > allowed:
        return f"speed {measured:.0f} is not within {pct:.0%} of {target:.0f}"
    else:
        return None


def forward_count(s):
    return (s["enc_left"] + s["enc_right"]) / 2


def speed_meter():
    prev = None
    prev_seq = None
    rover_time = 0.0

    def read(s):
        nonlocal prev, prev_seq, rover_time
        if s["seq"] == prev_seq:
            return None
        if prev_seq is not None:
            rover_time += rp.seq_delta(prev_seq, s["seq"]) * TELEMETRY_PERIOD_S
        curr = (forward_count(s), rover_time)
        speed = speed_between(prev, curr)
        prev = curr
        prev_seq = s["seq"]
        return speed
    return read

def cruise_speed(baseline, pct=SPEED_TOLERANCE, stopped_max=STOPPED_MAX,
                  readings=SETTLE_READINGS):
    meter = speed_meter()
    window = []

    def check(s):
        speed = meter(s)
        if speed is None:
            return "Nothing measured yet."
        if speed <= stopped_max:
            del window[:]
            return f"not moving forward yet (speed {speed:.0f})"
        window.append(speed)
        del window[:-readings]
        if len(window) < readings:
            return f"speed still changing ({speed:.0f})"
        average = sum(window) / len(window)
        if max(window) - min(window) > pct * abs(average):
            return f"speed still changing ({speed:.0f})"
        baseline[0] = average
        return None
    return check


def speed_follows(baseline, ratio, pct=SPEED_TOLERANCE, stopped_max=STOPPED_MAX):
    meter = speed_meter()

    def check(s):
        if baseline[0] is None:
            return "no cruise speed was recorded by an earlier step"
        return speed_problem(meter(s), baseline[0] * ratio, pct, stopped_max)
    return check


INDICATOR_COLORS = {
    rp.INDICATOR_OFF: "OFF",
    rp.INDICATOR_TELEOP: "BLUE",
    rp.INDICATOR_AUTONOMOUS: "RED",
    rp.INDICATOR_ARRIVED: "FLASHING GREEN",
    rp.INDICATOR_FAULT: "YELLOW",
}


class SequenceRecorder:
    COLUMNS = ("t_s", "step", "seq", "mode", "light", "cmd_status",
               "cmd_source",
               "drive_applied", "steer_applied", "speed_cps",
               "enc_left", "enc_right", "steer_fb",
               "cmd_age_ms", "enc_age_ms", "steer_age_ms",
               "controller_health", "feedback_flags", "faults")

    def __init__(self, path):
        self.path = path
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.COLUMNS)
        self.meter = speed_meter()
        self.started = time.monotonic()
        self.last_seq = None
        self.last_speed = None
        self.rows = 0

    def record(self, step_name, s):
        if s["seq"] == self.last_seq:
            return
        self.last_seq = s["seq"]
        speed = self.meter(s)
        if speed is not None:
            self.last_speed = speed
        self.writer.writerow((
            "%.3f" % (time.monotonic() - self.started),
            step_name,
            s["seq"],
            rp.MODE_NAMES.get(s["mode"], "?"),
            INDICATOR_COLORS.get(s["indicator_state"], "?"),
            rp.CMD_STATUS_NAMES.get(s["cmd_status"], "?"),
            rp.CMD_SOURCE_NAMES.get(s["cmd_source"], "?"),
            s["drive_applied"],
            s["steer_applied"],
            "" if speed is None else "%.0f" % speed,
            s["enc_left"],
            s["enc_right"],
            s["steer_fb"],
            s["cmd_age_ms"],
            s["encoder_age_ms"],
            s["steer_age_ms"],
            rp.CTRL_HEALTH_NAMES.get(s["controller_health"], "?"),
            "|".join(rp.feedback_flag_names(s["feedback_flags"])) or "none",
            "|".join(rp.fault_names(s["fault_status"])) or "none",
        ))
        self.rows += 1

    def close(self):
        self.file.close()


def build_mission():
    cruise = [None]

    auto = dict(mode=rp.MODE_AUTONOMOUS, c2_lost=False, from_autonomy=True)
    return [
        Step("1. start in SAFE",
             dict(drive_cmd=0, steer_cmd=0, mode=rp.MODE_SAFE,
                  c2_lost=False, from_autonomy=False),
             all_of(expect(mode=rp.MODE_SAFE,
                           cmd_status=rp.CMD_STATUS_STOPPED,
                           drive_applied=0, steer_applied=0,
                           indicator_state=rp.INDICATOR_OFF),
                    fault_clear(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),

        Step("1b. TELEOP shows blue",
             dict(drive_cmd=0, steer_cmd=0, mode=rp.MODE_TELEOP,
                  c2_lost=False, from_autonomy=False),
             expect(mode=rp.MODE_TELEOP,
                    indicator_state=rp.INDICATOR_TELEOP)),

        Step("1c. operator grants AUTONOMOUS",
             dict(drive_cmd=0, steer_cmd=0, stop=True, mode=rp.MODE_AUTONOMOUS,
                  c2_lost=False, from_autonomy=False),
             all_of(expect(mode=rp.MODE_AUTONOMOUS,
                           cmd_status=rp.CMD_STATUS_STOPPED,
                           indicator_state=rp.INDICATOR_AUTONOMOUS),
                    fault_clear(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),

        Step("2. receive AUTONOMOUS",
             dict(drive_cmd=0, steer_cmd=0, **auto),
             all_of(expect(mode=rp.MODE_AUTONOMOUS,
                           cmd_status=rp.CMD_STATUS_APPLIED,
                           indicator_state=rp.INDICATOR_AUTONOMOUS,
                           cmd_source=rp.CMD_SOURCE_AUTONOMY),
                    feedback_is_fresh())),

        Step("3. forward: speed rises, settles",
             dict(drive_cmd=FAST, steer_cmd=0, **auto),
             all_of(expect(cmd_status=rp.CMD_STATUS_APPLIED,
                           drive_applied=FAST, steer_applied=0),
                    cruise_speed(cruise),
                    feedback_is_fresh())),

        Step("4. change steering",
             dict(drive_cmd=FAST, steer_cmd=TURN, **auto),
             all_of(expect(cmd_status=rp.CMD_STATUS_APPLIED,
                           drive_applied=FAST, steer_applied=TURN),
                    steer_feedback_within(TURN, 25))),

        Step("5. slow to CRUISE: speed follows",
             dict(drive_cmd=CRUISE, steer_cmd=TURN, **auto),
             all_of(expect(cmd_status=rp.CMD_STATUS_APPLIED,
                           drive_applied=CRUISE, steer_applied=TURN),
                    speed_follows(cruise, CRUISE / FAST))),

        Step("6. STOP: speed returns to zero",
             dict(drive_cmd=CRUISE, steer_cmd=TURN, stop=True, **auto),
             all_of(expect(cmd_status=rp.CMD_STATUS_STOPPED,
                           drive_applied=0, steer_applied=0,
                           indicator_state=rp.INDICATOR_AUTONOMOUS),
                    speed_follows(cruise, 0))),

        Step("7. report TARGET_REACHED",
             dict(drive_cmd=0, steer_cmd=0, stop=True,
                  indicator_request=rp.INDICATOR_ARRIVED, **auto),
             expect(indicator_state=rp.INDICATOR_ARRIVED,
                    cmd_status=rp.CMD_STATUS_STOPPED,
                    drive_applied=0)),

        Step("8. return to SAFE",
             dict(drive_cmd=0, steer_cmd=0, mode=rp.MODE_SAFE,
                  c2_lost=False, from_autonomy=False),
             all_of(expect(mode=rp.MODE_SAFE,
                           cmd_status=rp.CMD_STATUS_STOPPED,
                           drive_applied=0, steer_applied=0,
                           indicator_state=rp.INDICATOR_OFF),
                    fault_clear(rp.FAULT_CMD_SOURCE, "CMD_SOURCE"))),
    ]


FEEDBACK_FIELDS = (
    "mode",
    "cmd_source",
    "drive_applied",
    "enc_left",
    "enc_right",
    "steer_applied",
    "steer_fb",
    "cmd_age_ms",
    "encoder_age_ms",
    "steer_age_ms",
    "controller_health",
    "cmd_status",
    "fault_status",
    "feedback_flags",
    "indicator_state",
)


def run_mission(link, steps=None, verbose=True, settle_s=0.4, recorder=None):
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

    try:
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
                    if recorder is not None:
                        recorder.record(step.name, snapshot)
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
                if snapshot is not None:
                    speed = recorder.last_speed if recorder is not None else None
                    print("       light=%-14s speed=%s" % (
                        INDICATOR_COLORS.get(snapshot["indicator_state"], "?"),
                        "?" if speed is None else "%.0f counts/s" % speed))
            if not ok:
                break
    finally:
        for _ in range(3):
            jl.send_control(drive_cmd=0, steer_cmd=0, mode=rp.MODE_SAFE,
                            c2_lost=False, stop=True)
            jl.poll()
            time.sleep(period)

    return report


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", action="store_true",
                    help="run against the compiled C++ simulator (no hardware)")
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyACM0 or COM5")
    ap.add_argument("--baud", type=int, default=SERIAL_BAUD_HZ)
    ap.add_argument("--record", default="closed_loop_run.csv",
                    help="CSV file to record every telemetry snapshot to")
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
    recorder = SequenceRecorder(args.record)
    try:
        report = run_mission(link, recorder=recorder)
    finally:
        link.close()
        recorder.close()

    print()
    last = report.results[-1].snapshot if report.results else None
    if last is not None:
        missing = [f for f in FEEDBACK_FIELDS if f not in last]
        print("feedback loop: %s" % ("every required field reached the Jetson"
                                     if not missing else "MISSING %s" % missing))
        print("  mode=%s  light=%s  applied=(%+d,%+d)  health=%s  age=%sms  faults=%s"
              % (rp.MODE_NAMES.get(last["mode"], "?"),
                 INDICATOR_COLORS.get(last["indicator_state"], "?"),
                 last["drive_applied"], last["steer_applied"],
                 rp.CTRL_HEALTH_NAMES.get(last["controller_health"], "?"),
                 last["cmd_age_ms"],
                 ",".join(rp.fault_names(last["fault_status"])) or "none"))
        print("  source=%s  feedback age enc=%sms steer=%sms  flags=%s"
              % (rp.CMD_SOURCE_NAMES.get(last["cmd_source"], "?"),
                 last["encoder_age_ms"], last["steer_age_ms"],
                 ",".join(rp.feedback_flag_names(last["feedback_flags"]))
                 or "none"))
    print("snapshots assembled: %d" % report.snapshots_seen)
    print("recorded %d snapshots to %s" % (recorder.rows, recorder.path))
    print()

    if report.ok:
        print("PASS: an autonomy decision travelled the Task 5 closed-loop protocol and")
        print("reached the drive stage; the controller acknowledged each")
        print("command by sequence number;")
        return 0
    print("FAIL at: %s" % ", ".join(r.name for r in report.failed))
    return 1


if __name__ == "__main__":
    sys.exit(main())
