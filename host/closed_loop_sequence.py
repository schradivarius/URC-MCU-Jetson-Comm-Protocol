"""
closed_loop_sequence.py

Modified autonomy_sim.py that executes a full closed loop sequence instead of the task 3 control and autonomy state validation

    python3 host/closed_loop_sequence.py --mock            # against the C++ simulator
    python3 host/closed_loop_sequence.py --channel vcan0   # against a virtual CAN bus
    python3 host/closed_loop_sequence.py --channel can0    # against the real Teensy

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

# can_link, rover_protocol, jetson_test and rover_config belong to the protocol
# repository, not to the autonomy program. In a shared checkout they sit right
# beside this file; when the autonomy program lives in a repository of its own,
# ROVER_PROTOCOL_ROOT names the protocol checkout they live in. Importing them
# either way is deliberate: this program must speak the protocol the firmware
# was compiled from, never a second copy of it.
_PROTOCOL_ROOT = os.environ.get("ROVER_PROTOCOL_ROOT")
if _PROTOCOL_ROOT:
    sys.path.insert(0, os.path.join(
        os.path.abspath(os.path.expanduser(_PROTOCOL_ROOT)), "host"))
else:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink, open_link  # noqa: E402
from rover_config import CONTROL_RATE_HZ, TELEMETRY_PERIOD_MS  # noqa: E402

# Half the sequence space. A snapshot's ack_seq counts as "at or after" the
# step's first frame when it is within this distance ahead, which is correct
# across the 255->0 wrap and cannot mistake an old frame for a new one.
SEQ_LOOKAHEAD = 128

# Mission speeds, in tenths of a percent of full effort. Modest on purpose:
# this runs against a simulator today and a real rover later, and a script
# nobody is afraid of is a script that gets run.
CRUISE = 400
TURN = 300
FAST = 500


def seq_at_or_after(ack_seq, first_seq):
    """Has `ack_seq` reached `first_seq`? Correct across the 255->0 wrap."""
    return rp.seq_delta(first_seq, ack_seq) < SEQ_LOOKAHEAD


class Step:
    """One decision: a command to hold, and what must become true because of it.

    `command` is whatever JetsonLink.send_control takes. `check` is given the
    snapshot and returns None when satisfied, or a string saying what is still
    wrong -- which becomes the failure message, so it is worth writing as the
    thing a human would want to read at 2 a.m.
    """

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


# ---------------------------------------------------------------------------
# Checks. Each is a closure over what the step expects, so a failure message
# names the actual numbers rather than "assertion failed".
# ---------------------------------------------------------------------------

def expect(**wanted):
    """Every named snapshot field must equal the given value."""
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
    """The steering actuator has actually moved, not just been commanded.

    `steer_applied` says the controller accepted the command; `steer_fb` is the
    plant answering. Both matter, and only together do they say the command
    reached the hardware.
    """
    def check(s):
        if abs(s["steer_fb"] - target) <= tolerance:
            return None
        return "steer_fb=%d has not reached %d" % (s["steer_fb"], target)
    return check


# ---------------------------------------------------------------------------
# Measured speed. The controller reports encoder COUNTS, not a speed, so the
# Jetson works the speed out itself: how far the count moved between two
# snapshots, divided by the time between them. This is the "check what
# actually happened" half of the closed loop.
# ---------------------------------------------------------------------------

# How close counts as "close enough", and how slow counts as "stopped".
# PLACEHOLDERS: the right values depend on how noisy the real encoders are.
SPEED_TOLERANCE = 0.10          # within 10% of the target
STOPPED_MAX = 200               # counts/sec

# The rover sends one snapshot every TELEMETRY_PERIOD_MS and numbers each one,
# so the time between two snapshots is (how many numbers apart) x 50 ms -- the
# ROVER's clock. Timing them by when the Jetson happened to read them does not
# work: two snapshots can be read 1 ms apart even though they were sent 50 ms
# apart, and 1200 counts / 0.001 s would read as 1,200,000 counts/sec.
TELEMETRY_PERIOD_S = TELEMETRY_PERIOD_MS / 1000.0


def speed_between(prev, curr):
    """prev and curr are each a tuple: (encoder_count, time_in_seconds).
    prev may be None. Return speed in counts/sec, or None if it can't be known."""
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
    """Return None if `measured` is close enough to `target`,
    otherwise a string saying what's wrong."""
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
    """Average of the two encoders. A turn speeds one side up and slows the
    other down by the same amount, so the average is the rover's forward
    motion with the turn cancelled out."""
    return (s["enc_left"] + s["enc_right"]) / 2


def speed_meter():
    """The memory ("backpack") from speed_check, on its own: feed it
    snapshots, get a speed back. None on the first snapshot, and on a repeat
    of the last one -- the same postcard twice says nothing new."""
    prev = None        # (count, time) of the last NEW postcard
    prev_seq = None    # its sequence number
    rover_time = 0.0   # seconds on the rover's clock since the first postcard

    def read(s):
        nonlocal prev, prev_seq, rover_time
        if s["seq"] == prev_seq:
            return None
        if prev_seq is not None:
            # seq_delta handles the 255 -> 0 wrap.
            rover_time += rp.seq_delta(prev_seq, s["seq"]) * TELEMETRY_PERIOD_S
        curr = (forward_count(s), rover_time)
        speed = speed_between(prev, curr)
        prev = curr
        prev_seq = s["seq"]
        return speed
    return read


def cruise_speed(baseline, pct=SPEED_TOLERANCE, stopped_max=STOPPED_MAX):
    """Passes once the rover is moving and its speed has settled, and puts
    that speed in baseline[0] for later steps to compare against.

    "Settled" means two readings in a row agree within pct. With today's
    simulator that is immediate; once Task 5.1 makes the rover speed up
    gradually, this waits out the ramp instead of grabbing a half-way speed.
    """
    meter = speed_meter()
    last_speed = None

    def check(s):
        nonlocal last_speed
        speed = meter(s)
        if speed is None:
            return "Nothing measured yet."
        if abs(speed) <= stopped_max:
            last_speed = None
            return f"not moving yet (speed {speed:.0f})"
        previous, last_speed = last_speed, speed
        if previous is None or speed_problem(speed, previous, pct, stopped_max):
            return f"speed still changing ({speed:.0f})"
        baseline[0] = speed
        return None
    return check


def speed_follows(baseline, ratio, pct=SPEED_TOLERANCE, stopped_max=STOPPED_MAX):
    """The measured speed is about `ratio` times the cruise speed an earlier
    step recorded.

    We command EFFORT, not speed, and don't know how one becomes the other,
    so the rover is compared against itself: cutting effort from 500 to 400
    should cut the speed to about 400/500 = 0.8 of what it was. ratio=0 means
    "stopped", which speed_problem judges with stopped_max instead of a %.
    """
    meter = speed_meter()

    def check(s):
        if baseline[0] is None:
            return "no cruise speed was recorded by an earlier step"
        return speed_problem(meter(s), baseline[0] * ratio, pct, stopped_max)
    return check


# ---------------------------------------------------------------------------
# Recording. Task 5.7: "The Jetson-side test program should record the full
# sequence."
# ---------------------------------------------------------------------------

# What the rear status light shows for each INDICATOR_* code (PROTOCOL.md
# 3.5.1). FAULT's yellow is the team's placeholder, not a URC rule.
INDICATOR_COLORS = {
    rp.INDICATOR_OFF: "OFF",
    rp.INDICATOR_TELEOP: "BLUE",
    rp.INDICATOR_AUTONOMOUS: "RED",
    rp.INDICATOR_ARRIVED: "FLASHING GREEN",
    rp.INDICATOR_FAULT: "YELLOW",
}


class SequenceRecorder:
    """Writes the run to a CSV file: one row for each new snapshot the
    mission reads (about 20 a second), tagged with the step that was running.
    Open it in a spreadsheet and it reads as the mission, in order."""

    COLUMNS = ("t_s", "step", "seq", "mode", "light", "cmd_status",
               "drive_applied", "steer_applied", "speed_cps",
               "enc_left", "enc_right", "steer_fb",
               "cmd_age_ms", "controller_health", "faults")

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
            return                      # already wrote this snapshot
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
            s["drive_applied"],
            s["steer_applied"],
            "" if speed is None else "%.0f" % speed,
            s["enc_left"],
            s["enc_right"],
            s["steer_fb"],
            s["cmd_age_ms"],
            rp.CTRL_HEALTH_NAMES.get(s["controller_health"], "?"),
            "|".join(rp.fault_names(s["fault_status"])) or "none",
        ))
        self.rows += 1

    def close(self):
        self.file.close()


# ---------------------------------------------------------------------------
# The mission
# ---------------------------------------------------------------------------

def build_mission():
    """The Task 5.7 closed-loop sequence.

    c2_lost is False on every step: a lost C2 link stops a TELEOP rover, and
    nothing here is about C2.

    The speed steps compare the rover against ITSELF. Step 3 records the speed
    it settles at under FAST; steps 5 and 6 check the speed is the right
    fraction of that. See speed_follows() for why.
    """
    # One-slot box: step 3 puts the cruise speed in, steps 5 and 6 read it.
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

        Step("2. receive AUTONOMOUS",
             dict(drive_cmd=0, steer_cmd=0, **auto),
             expect(mode=rp.MODE_AUTONOMOUS,
                    cmd_status=rp.CMD_STATUS_APPLIED,
                    # Red, per rule 1.e.ii, the moment the rover is
                    # autonomous -- not once it starts moving.
                    indicator_state=rp.INDICATOR_AUTONOMOUS)),

        Step("3. forward: speed rises, settles",
             dict(drive_cmd=FAST, steer_cmd=0, **auto),
             all_of(expect(cmd_status=rp.CMD_STATUS_APPLIED,
                           drive_applied=FAST, steer_applied=0),
                    cruise_speed(cruise))),

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
             # Still commanding CRUISE: STOP has to override an active
             # movement command, not just a zero one.
             dict(drive_cmd=CRUISE, steer_cmd=TURN, stop=True, **auto),
             # The light stays red: a paused autonomous rover is still under
             # autonomous operation.
             all_of(expect(cmd_status=rp.CMD_STATUS_STOPPED,
                           drive_applied=0, steer_applied=0,
                           indicator_state=rp.INDICATOR_AUTONOMOUS),
                    speed_follows(cruise, 0))),

        Step("7. report TARGET_REACHED",
             dict(drive_cmd=0, steer_cmd=0, stop=True,
                  indicator_request=rp.INDICATOR_ARRIVED, **auto),
             # Flashing green. Rule 1.e.xvii scores a target only on stopping
             # within 1 m AND indicating arrival, so this step is worth points
             # rather than just being tidy.
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


# Every field the task's feedback loop requires, and where it rides.
FEEDBACK_FIELDS = (
    "mode",               # current mode              (TELEM_STATE)
    "drive_applied",      # accepted drive command    (TELEM_CMD_ECHO)
    "steer_applied",      # accepted steering command (TELEM_CMD_ECHO)
    "controller_health",  # controller-health state   (TELEM_STATE)
    "cmd_age_ms",         # command age               (TELEM_DRIVE_R)
    "fault_status",       # fault state               (TELEM_POWER)
    "enc_left",           # measured drive            (TELEM_DRIVE_L)
    "enc_right",          # measured drive            (TELEM_DRIVE_R)
    "indicator_state",    # rear status light         (TELEM_STATE)
)


def run_mission(link, steps=None, verbose=True, settle_s=0.4, recorder=None):
    """Drive `steps` to completion against `link`. Returns a MissionReport.

    Stops at the first failure: once a step's precondition never arrived, the
    ones after it would be testing something other than what they claim.

    `recorder`, if given, is handed every snapshot (see SequenceRecorder).
    """
    jl = JetsonLink(link)
    report = MissionReport()
    period = 1.0 / CONTROL_RATE_HZ
    steps = build_mission() if steps is None else steps

    # Let the controller come up and the first snapshot assemble before the
    # first step starts timing. Without it, step 1 spends its budget waiting
    # for the simulator to launch and fails for a reason that is not its own.
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
                if recorder is not None:
                    recorder.record(step.name, snapshot)
                # Only a snapshot acknowledging a frame from THIS step can say
                # anything about this step. Anything older is the previous
                # command still echoing back.
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

    return report


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", action="store_true",
                    help="run against the compiled C++ simulator (no hardware)")
    ap.add_argument("--channel", help="CAN interface, e.g. can0 or vcan0")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--record", default="closed_loop_run.csv",
                    help="CSV file to record every telemetry snapshot to")
    args = ap.parse_args()

    if not args.mock and not args.channel:
        print("Specify --mock for the simulator, or --channel can0 for a real bus.",
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
