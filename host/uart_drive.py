#!/usr/bin/env python3
"""
uart_drive.py -- send drive commands to the MCU over USB serial and watch what
it does with them.

This is the laptop end of the UART bring-up: it proves the MCU receives a
command, decides whether it is allowed to act on it, and acts. Works on
Windows, macOS and Linux; the only dependency is pyserial.

    python host/uart_drive.py --list-ports
    python host/uart_drive.py --port COM5                       # zero command
    python host/uart_drive.py --port COM5 --drive 300 --seconds 3
    python host/uart_drive.py --port COM5 --drive 300 --steer -200 --seconds 5
    python host/uart_drive.py --port COM5 --stop

--drive and --steer are tenths of a percent of full effort, -1000..1000, the
same units as the protocol's drive_cmd and steer_cmd.

WHY IT SENDS CONTINUOUSLY
  The firmware stops the drives if no valid command arrives within
  WATCHDOG_TIMEOUT_MS. That is the safety property, not an inconvenience: a
  one-shot "move" command would be a rover that keeps driving after the laptop
  is unplugged. So this sends at CONTROL_RATE_HZ for --seconds and then stops.

EVERY EXIT PATH STOPS THE ROVER
  Normal finish, Ctrl-C, a decode error, an unplugged cable: the stop frames in
  the `finally` block are sent either way, as a continuation of the same
  sequence numbers, so the firmware sees one unbroken session. If the link is
  already gone they cannot arrive, which is exactly what the firmware's own
  watchdog is for.

TELEOP AND SAFE ONLY
  AUTONOMOUS is deliberately not offered. The firmware refuses an operator
  command in AUTONOMOUS, and the only way round that is for this script to
  claim its commands came from the autonomy stack -- which is the one lie the
  command-authority check exists to catch.

HOW IT DECIDES THE MCU ANSWERED
  Every TELEM_CMD_ECHO frame names the CONTROL sequence number it is
  acknowledging. An echo only counts if it acknowledges a frame THIS run sent
  and the command it describes is fresh, so a board still reporting a previous
  session's command is never mistaken for one that heard this one.

EXIT STATUS
  0  this run's commands were acknowledged, acted on if motion was asked for,
     and the stop on the way out was confirmed
  1  nothing decodable came back -- wrong port, wrong firmware, or no power
  2  telemetry arrived but never acknowledged this run's commands as wanted
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rover_protocol as rp  # noqa: E402
import serial_link  # noqa: E402
from rover_config import (CMD_MAX, CMD_MIN, CONTROL_RATE_HZ, MODE_SAFE,  # noqa: E402
                          MODE_TELEOP, SERIAL_BAUD_HZ, WATCHDOG_TIMEOUT_MS)

MODE_BY_NAME = {"safe": MODE_SAFE, "teleop": MODE_TELEOP}

STOP_FRAMES = 10
STATUS_PERIOD_S = 0.25


class Telemetry:
    """The latest value of every telemetry field, plus what this run proved."""

    def __init__(self):
        self.fields = {}
        self.frames = 0
        self.decode_errors = 0
        self.sent_seqs = set()
        self.acknowledged = False
        self.moved = False
        self.stopped = False

    def ingest(self, can_id, payload):
        decoder = rp.DECODERS.get(can_id)
        if decoder is None:
            return
        result, msg = decoder(payload)
        if result != rp.DECODE_OK or msg is None:
            self.decode_errors += 1
            return
        self.frames += 1
        self.fields.update(msg)
        if can_id == rp.CAN_ID_TELEM_CMD_ECHO:
            self._echo(msg)

    def _echo(self, msg):
        age = self.fields.get("cmd_age_ms", rp.CMD_AGE_UNKNOWN)
        fresh = age != rp.CMD_AGE_UNKNOWN and age < WATCHDOG_TIMEOUT_MS
        if not fresh or msg["ack_seq"] not in self.sent_seqs:
            return
        self.acknowledged = True
        status = msg["cmd_status"]
        if status == rp.CMD_STATUS_APPLIED and (msg["drive_applied"]
                                                or msg["steer_applied"]):
            self.moved = True
        if status == rp.CMD_STATUS_STOPPED:
            self.stopped = True

    def line(self):
        f = self.fields
        if not f:
            return "waiting for telemetry..."
        faults = rp.fault_names(f.get("fault_status", 0)) or ["none"]
        age = f.get("cmd_age_ms", rp.CMD_AGE_UNKNOWN)
        return ("mode=%-6s link=%-8s cmd_age=%-6s applied=%+5d/%+5d "
                "status=%-12s enc=%+8d/%+8d %5.2fV %5.2fA health=%-8s faults=%s"
                % (rp.MODE_NAMES.get(f.get("mode"), "?"),
                   rp.LINK_NAMES.get(f.get("jetson_link"), "?"),
                   "n/a" if age == rp.CMD_AGE_UNKNOWN else "%dms" % age,
                   f.get("drive_applied", 0), f.get("steer_applied", 0),
                   rp.CMD_STATUS_NAMES.get(f.get("cmd_status"), "?"),
                   f.get("enc_left", 0), f.get("enc_right", 0),
                   f.get("voltage_v", 0.0), f.get("current_a", 0.0),
                   rp.CTRL_HEALTH_NAMES.get(f.get("controller_health"), "?"),
                   ",".join(faults)))


class Sender:
    """Owns the CONTROL sequence number for the whole run.

    One counter across every phase, so the stop frames on the way out are the
    next frames of the same session rather than a restart at zero, which the
    firmware would rightly report as a sequence gap.
    """

    def __init__(self, link, telemetry, rate_hz, quiet):
        self.link = link
        self.telemetry = telemetry
        self.period = 1.0 / rate_hz
        self.quiet = quiet
        self.seq = 0
        self.sent = 0

    def run(self, drive, steer, mode, stop, seconds, label):
        """Send one command continuously for `seconds`, draining telemetry.

        The send clock resynchronises rather than catching up: a slow console
        or a stalled write must not turn into a burst of backdated frames.
        """
        deadline = time.monotonic() + seconds
        next_send = time.monotonic()
        next_status = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.telemetry.sent_seqs.add(self.seq)
                self.link.send(rp.CAN_ID_CONTROL,
                               rp.encode_control(drive, steer, mode,
                                                 c2_lost=False, stop=stop,
                                                 seq=self.seq))
                self.seq = (self.seq + 1) & 0xFF
                self.sent += 1
                next_send += self.period
                if next_send < now:
                    next_send = now + self.period
            for can_id, payload in self.link.recv():
                self.telemetry.ingest(can_id, payload)
            if not self.quiet and now >= next_status:
                print("%-5s %s" % (label, self.telemetry.line()))
                next_status = now + STATUS_PERIOD_S
            time.sleep(0.002)


def command_range(value):
    number = int(value)
    if not CMD_MIN <= number <= CMD_MAX:
        raise argparse.ArgumentTypeError(
            "must be %d..%d; the firmware REJECTS an out-of-range frame "
            "outright, so a bigger number does not drive harder, it stops the "
            "rover" % (CMD_MIN, CMD_MAX))
    return number


def show_ports():
    ports = serial_link.list_ports()
    if not ports:
        print("no serial ports found; is the Teensy plugged in and flashed?")
        return 1
    print("serial ports:")
    for device, description in ports:
        print("  %-20s %s" % (device, description))
    print("\npass one as --port")
    return 0


def verdict(telemetry, moving, stopping):
    if telemetry.frames == 0:
        print("\nFAIL: nothing came back. Check --port, that the firmware was "
              "built for the serial link (pio run -e teensy41), and power.")
        return 1
    if not telemetry.acknowledged:
        print("\nFAIL: the MCU is sending telemetry but never acknowledged a "
              "command from this run. Its last status is above.")
        return 2
    if moving and not telemetry.moved:
        print("\nFAIL: the MCU acknowledged this run's commands but never "
              "applied a non-zero output. STOPPED means a stop or a mode that "
              "forbids motion; WRONG_SOURCE means a command-authority "
              "mismatch.")
        return 2
    if moving and not telemetry.stopped:
        print("\nFAIL: the MCU applied the command but never confirmed the "
              "stop sent on exit. Check the motors are not still running; the "
              "firmware watchdog should have stopped them within %d ms."
              % WATCHDOG_TIMEOUT_MS)
        return 2
    if stopping and not telemetry.stopped:
        print("\nFAIL: the MCU acknowledged this run's commands but never "
              "reported itself STOPPED.")
        return 2
    if moving:
        print("\nOK: the MCU received the commands and acted on them.")
    elif stopping:
        print("\nOK: the MCU received the stop and is holding the drives at zero.")
    else:
        print("\nOK: the MCU received the commands.")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Drive the rover MCU over USB serial.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Units: --drive/--steer are tenths of a percent, -1000..1000.")
    parser.add_argument("--list-ports", action="store_true",
                        help="print the serial ports this machine can see")
    parser.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD_HZ)
    parser.add_argument("--drive", type=command_range, default=0,
                        help="forward effort, -1000..1000 (default 0)")
    parser.add_argument("--steer", type=command_range, default=0,
                        help="turn effort, -1000..1000 (default 0)")
    parser.add_argument("--seconds", type=float, default=3.0,
                        help="how long to hold the command (default 3)")
    parser.add_argument("--mode", choices=sorted(MODE_BY_NAME), default="teleop",
                        help="commanded mode (default teleop)")
    parser.add_argument("--stop", action="store_true",
                        help="send the stop flag instead of a command")
    parser.add_argument("--rate", type=float, default=CONTROL_RATE_HZ,
                        help="command rate in Hz (default %d)" % CONTROL_RATE_HZ)
    parser.add_argument("--quiet", action="store_true",
                        help="only print the summary")
    args = parser.parse_args()

    if args.list_ports:
        return show_ports()
    if not args.port:
        parser.error("--port is required (try --list-ports)")
    if args.rate <= 0:
        parser.error("--rate must be positive")
    if 1000.0 / args.rate > WATCHDOG_TIMEOUT_MS / 2.0:
        parser.error("--rate %g Hz is slower than half the firmware's %d ms "
                     "command watchdog; the rover would keep stopping itself"
                     % (args.rate, WATCHDOG_TIMEOUT_MS))
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    mode = MODE_BY_NAME[args.mode]
    if args.stop and (args.drive or args.steer):
        parser.error("--stop cannot be combined with --drive or --steer")
    if (args.drive or args.steer) and mode == MODE_SAFE:
        parser.error("--mode safe cannot move; SAFE means stopped on purpose")
    moving = bool(args.drive or args.steer)
    stopping = args.stop or mode == MODE_SAFE

    try:
        link = serial_link.SerialLink(args.port, baud=args.baud)
    except ImportError:
        print("pyserial is not installed: python -m pip install pyserial",
              file=sys.stderr)
        return 1
    except Exception as exc:
        print("could not open %s: %s" % (args.port, exc), file=sys.stderr)
        return 1

    telemetry = Telemetry()
    sender = Sender(link, telemetry, args.rate, args.quiet)
    try:
        if args.stop:
            print("sending STOP on %s" % args.port)
            sender.run(0, 0, MODE_SAFE, True, args.seconds, "stop")
        else:
            print("sending drive=%+d steer=%+d mode=%s for %.1fs on %s"
                  % (args.drive, args.steer, args.mode, args.seconds, args.port))
            sender.run(args.drive, args.steer, mode, False, args.seconds, "run")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        try:
            sender.run(0, 0, MODE_SAFE, True, STOP_FRAMES / args.rate, "stop")
        except Exception as exc:
            print("could not send the stop frames: %s" % exc, file=sys.stderr)
        link.close()

    print("\n%s" % telemetry.line())
    print("sent %d command frames, decoded %d telemetry frames "
          "(%d undecodable, %d dropped by framing)"
          % (sender.sent, telemetry.frames, telemetry.decode_errors,
             link.frames_dropped))
    return verdict(telemetry, moving, stopping)


if __name__ == "__main__":
    sys.exit(main())
