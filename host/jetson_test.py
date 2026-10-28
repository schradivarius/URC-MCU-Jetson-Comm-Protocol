#!/usr/bin/env python3
"""
jetson_test.py -- Jetson-side test harness for the rover CAN link.

    # Against the compiled simulator (no hardware, no CAN drivers):
    python3 host/jetson_test.py --mock --duration 5

    # Against a Linux virtual CAN bus:
    #   sudo modprobe vcan && sudo ip link add dev vcan0 type vcan
    #   sudo ip link set up vcan0
    python3 host/jetson_test.py --channel vcan0

    # Against the real Teensy on a real bus:
    python3 host/jetson_test.py --channel can0 --bitrate 500000

Sends CONTROL at 20 Hz with an incrementing sequence number, and reassembles
the six telemetry frames into coherent snapshots.

Declares the LINK down if no telemetry has arrived within LINK_TIMEOUT_S --
separate from, and no substitute for, the controller's own command-age
watchdog. The rover's safety never depends on the Jetson noticing anything.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import c2_link  # noqa: E402
import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from rover_config import (  # noqa: E402
    CAN_BITRATE_HZ, CONTROL_RATE_HZ, LINK_TIMEOUT_S,
)


# One row per snapshot, the same columns for every program that logs one, so
# a log from this file, autonomy_sim.py or the closed-loop sequence can be
# compared column for column. Covers every field Task 5.5 asks the MCU to
# send, plus the snapshot seq and the ack_seq that ties it to a command.
SNAPSHOT_COLUMNS = (
    "t_s", "seq", "ack_seq",
    "mode", "active_mode", "active_source",       # what was commanded vs done
    "drive_applied", "steer_applied",             # commanded drive / steering
    "enc_left", "enc_right", "steer_fb",          # measured drive / steering
    "stop", "autonomy_abort", "cmd_status",       # stop / abort state
    "cmd_age_ms", "enc_age_ms", "steer_age_ms",   # how old each value is
    "controller_health", "fault_status", "faults",
    "indicator_request", "indicator_state",
)


def snapshot_row(snapshot, t_s):
    """`snapshot` as a list in SNAPSHOT_COLUMNS order, ready for csv.writer.

    Codes are written as names where a name exists (TELEOP rather than 1),
    because a log is read by people. fault_status stays a number so it can
    still be masked; `faults` carries the same bits as names.
    """
    s = snapshot
    return [
        "%.3f" % t_s, s["seq"], s["ack_seq"],
        rp.MODE_NAMES.get(s["mode"], s["mode"]),
        rp.MODE_NAMES.get(s["active_mode"], s["active_mode"]),
        rp.ACTIVE_SOURCE_NAMES.get(s["active_source"], s["active_source"]),
        s["drive_applied"], s["steer_applied"],
        s["enc_left"], s["enc_right"], s["steer_fb"],
        int(s["stop"]), int(s["autonomy_abort"]),
        rp.CMD_STATUS_NAMES.get(s["cmd_status"], s["cmd_status"]),
        s["cmd_age_ms"], s["enc_age_ms"], s["steer_age_ms"],
        rp.CTRL_HEALTH_NAMES.get(s["controller_health"], s["controller_health"]),
        s["fault_status"], "|".join(rp.fault_names(s["fault_status"])),
        rp.INDICATOR_NAMES.get(s["indicator_request"], s["indicator_request"]),
        rp.INDICATOR_NAMES.get(s["indicator_state"], s["indicator_state"]),
    ]


def format_age(age_ms, stale=False):
    """An age field as text: `never`, `>65534ms` (saturated) or `NNms`, with
    ` STALE` appended when the matching stale fault bit is set."""
    text = ("never" if age_ms == rp.CMD_AGE_UNKNOWN
            else ">%dms" % rp.CMD_AGE_MAX if age_ms == rp.CMD_AGE_MAX
            else "%dms" % age_ms)
    return text + " STALE" if stale else text



class TelemetryAssembler:
    """Collects the six telemetry frames of one cycle into one snapshot.

    All six frames of a cycle carry the SAME sequence number, which is how a
    coherent snapshot is told from one torn across two cycles. A torn snapshot
    is counted rather than silently published, because mixing a fresh encoder
    reading with a stale fault word is exactly the kind of quiet wrongness
    that makes a dashboard untrustworthy.
    """

    def __init__(self):
        self.partial = {}          # can_id -> decoded dict, current seq only
        self.partial_seq = None
        self.snapshot = None       # last COMPLETE snapshot
        self.snapshot_seq = None
        self.complete_count = 0
        self.torn_count = 0
        self.frames_by_id = {cid: 0 for cid in rp.TELEM_IDS}
        self.decode_errors = {}    # DecodeResult -> count
        # Called with every complete snapshot as it is assembled. One poll()
        # can complete more than one, and `snapshot` only keeps the last, so
        # a logger that wants every row hooks in here.
        self.on_snapshot = None

    def missing_ids(self):
        """Telemetry ids nothing has ever arrived on.

        Worth naming rather than just waiting: a controller running an older
        protocol sends a SUBSET of these frames, so the snapshot never
        completes and the only symptom is silence. There is no version field
        on the wire (PROTOCOL.md 6), so which ids showed up is the diagnostic.
        """
        return tuple(cid for cid, n in self.frames_by_id.items() if n == 0)

    def feed(self, can_id, frame):
        decoder = rp.DECODERS.get(can_id)
        if decoder is None:
            return False           # not telemetry; caller counts it
        result, decoded = decoder(frame)
        if result != rp.DECODE_OK:
            self.decode_errors[result] = self.decode_errors.get(result, 0) + 1
            return True
        self.frames_by_id[can_id] += 1
        seq = decoded["seq"]

        if self.partial_seq is None or seq != self.partial_seq:
            # A new cycle started. If the previous one never completed, the
            # snapshot was torn -- frames were lost mid-cycle.
            if self.partial_seq is not None and len(self.partial) < len(rp.TELEM_IDS):
                self.torn_count += 1
            self.partial = {}
            self.partial_seq = seq

        self.partial[can_id] = decoded
        if len(self.partial) == len(rp.TELEM_IDS):
            merged = {"seq": seq}
            for part in self.partial.values():
                merged.update({k: v for k, v in part.items() if k != "seq"})
            self.snapshot = merged
            self.snapshot_seq = seq
            self.complete_count += 1
            if self.on_snapshot is not None:
                self.on_snapshot(merged)
        return True


class JetsonLink:
    def __init__(self, link):
        self.link = link
        self.tele = TelemetryAssembler()
        self.frames_sent = 0
        self.unknown_frames = 0
        self.last_rx_time = None
        self._tx_seq = 0

    def send_control(self, drive_cmd, steer_cmd, mode, c2_lost, stop=False,
                     autonomy_abort=False, return_request=False,
                     indicator_request=rp.INDICATOR_OFF, from_autonomy=False):
        # c2_lost is required, not defaulted: see encode_control's docstring.
        # from_autonomy defaults to the operator, which is the fail-safe
        # direction -- a forgotten argument gets refused in AUTONOMOUS rather
        # than driving an autonomous rover.
        self._tx_seq = (self._tx_seq + 1) & 0xFF
        self.link.send(rp.CAN_ID_CONTROL,
                       rp.encode_control(drive_cmd, steer_cmd, mode, c2_lost,
                                         stop=stop,
                                         seq=self._tx_seq,
                                         autonomy_abort=autonomy_abort,
                                         return_request=return_request,
                                         indicator_request=indicator_request,
                                         from_autonomy=from_autonomy))
        self.frames_sent += 1
        return self._tx_seq

    @property
    def tx_seq(self):
        """The sequence number of the last CONTROL frame sent, for matching
        against the ack_seq the controller echoes back."""
        return self._tx_seq

    def poll(self):
        for can_id, frame in self.link.recv():
            if self.tele.feed(can_id, frame):
                self.last_rx_time = time.monotonic()
            else:
                # A shared bus carries motor-controller and payload traffic.
                # Counted rather than silently dropped, so an unexpected id is
                # visible instead of invisible.
                self.unknown_frames += 1

    @property
    def snapshot(self):
        return self.tele.snapshot

    @property
    def link_up(self):
        if self.last_rx_time is None:
            return False
        return (time.monotonic() - self.last_rx_time) < LINK_TIMEOUT_S

    def format_status(self, elapsed):
        s = self.snapshot
        if s is None:
            missing = self.tele.missing_ids()
            if missing and any(self.tele.frames_by_id.values()):
                # Some ids are arriving and others never have, which is what a
                # protocol-version mismatch looks like from this end.
                return ("[t=%5.1fs] no complete snapshot: nothing on %s"
                        % (elapsed, ", ".join("0x%03x" % c for c in missing)))
            return "[t=%5.1fs] waiting for a complete telemetry snapshot..." % elapsed
        faults = ",".join(rp.fault_names(s["fault_status"])) or "none"
        age_txt = format_age(s["cmd_age_ms"])
        enc_age_txt = format_age(s["enc_age_ms"],
                                 bool(s["fault_status"] & rp.FAULT_ENCODER_STALE))
        steer_age_txt = format_age(s["steer_age_ms"],
                                   bool(s["fault_status"] & rp.FAULT_STEER_STALE))
        flags = "".join(("S" if s["stop"] else "-",
                         "A" if s["autonomy_abort"] else "-"))
        return ("[t=%5.1fs] link=%-4s seq=%3d sent=%4d snap=%4d torn=%2d "
                "enc=(%+9d,%+9d) steer=%+5d %+7.2fA %6.2fV mode=%-10s "
                "active=%-10s src=%-9s jet=%-12s c2=%-12s ctrl=%-12s "
                "ind=%-10s req=%-10s age=%-8s enc_age=%-14s steer_age=%-14s "
                "stop/abort=%s cmd=%-12s ack=%3d applied=(%+5d,%+5d) faults=%s"
                % (elapsed, "UP" if self.link_up else "DOWN", s["seq"],
                   self.frames_sent, self.tele.complete_count,
                   self.tele.torn_count, s["enc_left"], s["enc_right"],
                   s["steer_fb"], s["current_a"], s["voltage_v"],
                   rp.MODE_NAMES.get(s["mode"], "?%d" % s["mode"]),
                   rp.MODE_NAMES.get(s["active_mode"], "?"),
                   rp.ACTIVE_SOURCE_NAMES.get(s["active_source"], "?"),
                   rp.LINK_NAMES.get(s["jetson_link"], "?"),
                   rp.LINK_NAMES.get(s["c2_link"], "?"),
                   rp.CTRL_HEALTH_NAMES.get(s["controller_health"], "?"),
                   rp.INDICATOR_NAMES.get(s["indicator_state"], "?"),
                   rp.INDICATOR_NAMES.get(s["indicator_request"], "?"),
                   age_txt, enc_age_txt, steer_age_txt, flags,
                   rp.CMD_STATUS_NAMES.get(s["cmd_status"], "?"), s["ack_seq"],
                   s["drive_applied"], s["steer_applied"], faults))


def run(link, duration_s, c2=None, verbose=True):
    jl = JetsonLink(link)
    # The controller has no radio, so the base-station link is ours to watch
    # and forward. Without a C2 link to watch we must report LOST, never OK:
    # "we did not look" is not "it is fine".
    c2_monitor = c2_link.C2Monitor()
    period = 1.0 / CONTROL_RATE_HZ
    t_start = time.monotonic()
    t_next = t_start
    last_printed = -1

    while (time.monotonic() - t_start) < duration_s:
        now = time.monotonic()
        if now >= t_next:
            elapsed = now - t_start
            drive = 300 if int(elapsed) % 4 < 2 else -300
            steer = int(200 * ((elapsed % 2) - 1))
            c2_lost = (True if c2 is None
                       else c2_link.poll_c2_lost(c2, c2_monitor, now))
            jl.send_control(drive, steer, rp.MODE_TELEOP, c2_lost,
                            indicator_request=rp.INDICATOR_TELEOP)
            t_next += period
            if now - t_next > period:
                t_next = now + period          # do not burst after a stall
        jl.poll()

        if verbose and jl.snapshot and jl.tele.complete_count != last_printed:
            last_printed = jl.tele.complete_count
            print("\r" + jl.format_status(now - t_start), end="", flush=True)
        time.sleep(0.005)

    if verbose:
        print()
        print("sent=%d complete_snapshots=%d torn=%d unknown_id=%d"
              % (jl.frames_sent, jl.tele.complete_count, jl.tele.torn_count,
                 jl.unknown_frames))
        print("frames per id: %s" % {hex(k): v for k, v in jl.tele.frames_by_id.items()})
        if jl.tele.decode_errors:
            print("WARNING decode errors: %s"
                  % {rp.DECODE_NAMES[k]: v for k, v in jl.tele.decode_errors.items()})
    return jl


def open_link(args):
    if args.mock:
        return can_link.SimLink()
    return can_link.SocketCanLink(channel=args.channel, bitrate=args.bitrate)


def open_c2_link(args):
    # Only the simulated base-station link exists so far; a real radio link
    # plugs in here with the same is_connected()/recv() shape.
    link = c2_link.SimC2Link()
    link.connect()
    return link


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", action="store_true",
                    help="run against the compiled C++ simulator (no hardware)")
    ap.add_argument("--channel", help="CAN interface, e.g. can0 or vcan0")
    ap.add_argument("--bitrate", type=int, default=CAN_BITRATE_HZ)
    ap.add_argument("--duration", type=float, default=10.0)
    args = ap.parse_args()

    if not args.mock and not args.channel:
        print("Specify --mock for the simulator, or --channel can0 for a real bus.",
              file=sys.stderr)
        sys.exit(1)

    link = open_link(args)
    c2 = open_c2_link(args)
    try:
        run(link, args.duration, c2=c2)
    finally:
        c2.disconnect()
        link.close()


if __name__ == "__main__":
    main()
