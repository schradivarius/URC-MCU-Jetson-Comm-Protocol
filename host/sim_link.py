import os
import queue
import shutil
import subprocess
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol_root  # noqa: E402

REPO = protocol_root.protocol_root()
SIM_SOURCES = [
    protocol_root.tools("rover_sim.cpp"),
    protocol_root.firmware_src("rover_controller.cpp"),
    protocol_root.firmware_src("rover_protocol.cpp"),
    protocol_root.firmware_src("rover_drive.cpp"),
    protocol_root.firmware_src("rover_drive_sim.cpp"),
    protocol_root.firmware_src("rover_encoder.cpp"),
    protocol_root.firmware_src("rover_steer_sim.cpp"),
    protocol_root.firmware_src("rover_feedback_faults.cpp"),
]
SIM_HEADERS = [
    protocol_root.firmware_src(name)
    for name in ("rover_config.h", "rover_protocol.h", "rover_controller.h",
                 "rover_drive.h", "rover_drive_sim.h", "rover_encoder.h",
                 "rover_steer_sim.h", "rover_feedback_faults.h")
]
SIM_BINARY = os.path.join(protocol_root.build_dir(), "rover_sim")


def build_sim(force=False):
    missing = [p for p in SIM_SOURCES + SIM_HEADERS if not os.path.exists(p)]
    if missing:
        hint = ("" if not protocol_root.is_overridden() else
                "; %s is set, and must point at the root of a "
                "protocol-repository checkout" % protocol_root.ENV_VAR)
        raise RuntimeError("firmware sources missing under %s: %s%s"
                           % (REPO, ", ".join(os.path.relpath(m, REPO)
                                              for m in missing), hint))
    if not force and os.path.exists(SIM_BINARY):
        newest_src = max(os.path.getmtime(s) for s in SIM_SOURCES + SIM_HEADERS)
        if os.path.getmtime(SIM_BINARY) >= newest_src:
            return SIM_BINARY
    cxx = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if cxx is None:
        raise RuntimeError("no C++ compiler found; the simulator is compiled from "
                           "firmware/src so it cannot drift from the firmware")
    os.makedirs(os.path.dirname(SIM_BINARY), exist_ok=True)
    subprocess.run([cxx, "-std=c++17", "-O1"] + SIM_SOURCES + ["-o", SIM_BINARY],
                   check=True, capture_output=True)
    return SIM_BINARY


class SimLink:
    def __init__(self, inject=None):
        argv = [build_sim()]
        if inject:
            argv += ["--inject", inject]
        self.proc = subprocess.Popen(
            argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        self._frames = queue.Queue()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        for line in self.proc.stdout:
            parts = line.split()
            if len(parts) == 2:
                try:
                    self._frames.put((int(parts[0], 16), bytes.fromhex(parts[1])))
                except ValueError:
                    pass

    def send(self, can_id, payload):
        try:
            self.proc.stdin.write("%03x %s\n" % (can_id, payload.hex()))
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    def recv(self):
        frames = []
        while True:
            try:
                frames.append(self._frames.get_nowait())
            except queue.Empty:
                return frames

    def close(self):
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self._reader.join(timeout=1)
        try:
            self.proc.stdout.close()
        except Exception:
            pass
