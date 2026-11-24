#!/usr/bin/env python3
"""
groupf_bench.py -- interactive command sender for Group F scope verification.

    python3 host/groupf_bench.py --port /dev/cu.usbmodemXXXX   # real Teensy (Mac)

Keeps sending CONTROL at 20 Hz with whatever you last chose. Commands:
    drive 500   TELEOP drive +500 -> pin 2 ~1750 us
    steer 300   change steer only (pin 4)
    stop        keep sending with STOP flag -> 1500 us
    pause       stop sending (timeout / Jetson loss)
    safe        send SAFE, clears a latched FAULT
    status      print latest telemetry
    quit        send SAFE briefly, then exit
"""

import argparse
import os
import queue
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink, open_link  # noqa: E402
from rover_config import SERIAL_BAUD_HZ, CONTROL_RATE_HZ  # noqa: E402


def stdin_reader(q):
    for raw in sys.stdin:
        q.put(raw.strip())
    q.put("quit")


def describe(state):
    if state["sending"] is False:
        return "PAUSED (nothing sent)"
    if state["mode"] == rp.MODE_SAFE:
        return "SAFE"
    return "TELEOP drive=%+d steer=%+d%s" % (
        state["drive"], state["steer"], " STOP" if state["stop"] else "")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", action="store_true", help="run against the simulator")
    ap.add_argument("--port", help="serial port, e.g. /dev/cu.usbmodem12345601")
    ap.add_argument("--baud", type=int, default=SERIAL_BAUD_HZ)
    args = ap.parse_args()
    if not args.mock and not args.port:
        import serial_link
        ports = serial_link.list_ports()
        print("Specify --port. Candidates: %s" % (", ".join(ports) or "none found"),
              file=sys.stderr)
        sys.exit(1)

    link = open_link(args)
    jl = JetsonLink(link)
    state = {"sending": True, "mode": rp.MODE_SAFE, "drive": 0, "steer": 0, "stop": False}
    cmds = queue.Queue()
    threading.Thread(target=stdin_reader, args=(cmds,), daemon=True).start()

    print("Connected. Sending SAFE. Type 'drive 500' to start; 'quit' to exit.")
    period = 1.0 / CONTROL_RATE_HZ
    t_next = time.monotonic()
    last_mode = None
    try:
        while True:
            try:
                line = cmds.get_nowait()
            except queue.Empty:
                line = None
            if line:
                parts = line.lower().split()
                word = parts[0]
                try:
                    if word == "drive":
                        state.update(sending=True, mode=rp.MODE_TELEOP,
                                     drive=int(parts[1]), stop=False)
                    elif word == "steer":
                        state.update(sending=True, mode=rp.MODE_TELEOP,
                                     steer=int(parts[1]), stop=False)
                    elif word == "stop":
                        state.update(sending=True, stop=True)
                    elif word == "pause":
                        state.update(sending=False)
                    elif word == "safe":
                        state.update(sending=True, mode=rp.MODE_SAFE,
                                     drive=0, steer=0, stop=False)
                    elif word == "status":
                        print(jl.format_status(0.0))
                        continue
                    elif word in ("quit", "exit", "q"):
                        break
                    else:
                        print("unknown command: %r" % line)
                        continue
                except (IndexError, ValueError):
                    print("usage: %s <number between -1000 and 1000>" % word)
                    continue
                print("%.3f  now sending: %s" % (time.monotonic(), describe(state)))

            now = time.monotonic()
            if now >= t_next:
                if state["sending"]:
                    jl.send_control(state["drive"], state["steer"], state["mode"],
                                    c2_lost=False, stop=state["stop"],
                                    indicator_request=(rp.INDICATOR_TELEOP
                                                       if state["mode"] == rp.MODE_TELEOP
                                                       else rp.INDICATOR_OFF))
                t_next += period
                if now - t_next > period:
                    t_next = now + period
            jl.poll()
            s = jl.snapshot
            if s is not None and s["mode"] != last_mode:
                last_mode = s["mode"]
                faults = ",".join(rp.fault_names(s["fault_status"])) or "none"
                print("%.3f  rover reports mode=%s applied=(%+d,%+d) faults=%s"
                      % (time.monotonic(), rp.MODE_NAMES.get(s["mode"], "?"),
                         s["drive_applied"], s["steer_applied"], faults))
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            jl.send_control(0, 0, rp.MODE_SAFE, c2_lost=False)
            time.sleep(period)
        link.close()
        print("Sent SAFE and closed the link.")


if __name__ == "__main__":
    main()
