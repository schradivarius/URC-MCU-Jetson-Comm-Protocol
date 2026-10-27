"""
serial_link.py -- the host's USB-serial / UART interface to the MCU.

Same three calls as can_link.py's backends, so anything written against one
works against the other:

    send(can_id, payload_bytes)
    recv() -> list of (can_id, payload_bytes) received since the last call
    close()

The wire format is SLIP (RFC 1055) framing around a body of
`id_low, id_high, <protocol frame bytes>`. The protocol's own sequence number
and CRC-8 are inside those bytes and are not duplicated here, so a frame that
survives framing is still checked by rover_protocol exactly as it would be off
a CAN bus. SLIP is used rather than a length-prefixed header because it
resynchronises on its own: any amount of noise, a half-written frame or a board
reset mid-transmission costs one frame, not the rest of the session.

Mirrors firmware/src/rover_serial.h. tests/host/test_serial_framing.py fails if
the two disagree.
"""

import glob
import os
import queue
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rover_config import SERIAL_BAUD_HZ  # noqa: E402

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

SERIAL_ID_BYTES = 2
FRAME_DLC = 8
SERIAL_BODY_MAX = SERIAL_ID_BYTES + FRAME_DLC
SERIAL_ENCODED_MAX = 2 + 2 * SERIAL_BODY_MAX

MAX_CAN_ID = 0x7FF


def encode_frame(can_id, payload):
    """SLIP-encode one (id, payload) pair. Raises on anything unsendable."""
    if not 0 <= can_id <= MAX_CAN_ID:
        raise ValueError("can_id %r is not an 11-bit identifier" % (can_id,))
    if len(payload) > FRAME_DLC:
        raise ValueError("payload is %d bytes, max %d" % (len(payload), FRAME_DLC))
    body = bytes((can_id & 0xFF, (can_id >> 8) & 0xFF)) + bytes(payload)
    out = bytearray([SLIP_END])
    for byte in body:
        if byte == SLIP_END:
            out += bytes((SLIP_ESC, SLIP_ESC_END))
        elif byte == SLIP_ESC:
            out += bytes((SLIP_ESC, SLIP_ESC_ESC))
        else:
            out.append(byte)
    out.append(SLIP_END)
    return bytes(out)


class FrameReader:
    """Byte-stream decoder. feed() returns the frames completed by this chunk.

    A body that is too short to hold an id plus one payload byte, one that
    overruns SERIAL_BODY_MAX, one whose id is not an 11-bit identifier, or an
    escape followed by a byte that is not a defined escape code, is counted in
    `dropped` and discarded. An empty body
    is not a drop: the encoder emits a leading END precisely so that whatever
    preceded a frame is flushed, which shows up here as an empty body.
    """

    def __init__(self):
        self._body = bytearray()
        self._escaped = False
        self._bad = False
        self.accepted = 0
        self.dropped = 0

    def feed(self, chunk):
        frames = []
        for byte in chunk:
            if byte == SLIP_END:
                frame = self._finish()
                if frame is not None:
                    frames.append(frame)
                continue
            if self._escaped:
                self._escaped = False
                if byte == SLIP_ESC_END:
                    byte = SLIP_END
                elif byte == SLIP_ESC_ESC:
                    byte = SLIP_ESC
                else:
                    self._bad = True
                    continue
            elif byte == SLIP_ESC:
                self._escaped = True
                continue
            if len(self._body) >= SERIAL_BODY_MAX:
                self._bad = True
                continue
            self._body.append(byte)
        return frames

    def _finish(self):
        body, bad = bytes(self._body), self._bad
        self._body = bytearray()
        self._escaped = False
        self._bad = False
        if bad or len(body) <= SERIAL_ID_BYTES:
            if bad or body:
                self.dropped += 1
            return None
        can_id = body[0] | (body[1] << 8)
        if can_id > MAX_CAN_ID:
            self.dropped += 1
            return None
        self.accepted += 1
        return (can_id, body[SERIAL_ID_BYTES:])


def list_ports():
    """Candidate serial ports, best-effort and never raising.

    pyserial's enumeration is the good answer and is tried first. The glob
    fallback exists because a Linux or macOS user without pyserial installed
    still deserves to be told what to pass to --port.
    """
    try:
        from serial.tools import list_ports as pyserial_ports
        return [(p.device, p.description or "") for p in pyserial_ports.comports()]
    except Exception:
        pass
    found = []
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*"):
        found += sorted(glob.glob(pattern))
    return [(path, "") for path in found]


class SerialLink:
    """pyserial on a USB-serial or UART port.

    Reading happens on a thread rather than by polling in recv(), for the same
    reason can_link.SimLink does it: pyserial buffers in userspace, so a poll
    that asks the OS whether bytes are waiting can report nothing while whole
    frames sit in the library's buffer.

    DTR is asserted explicitly on open. The firmware only transmits while the
    host has the port open, which it reads off DTR; pyserial asserts it on
    open, but a driver or a previous program can leave it low, and the symptom
    is a link that accepts commands and sends nothing back.
    """

    def __init__(self, port, baud=SERIAL_BAUD_HZ, timeout=0.05):
        import serial
        self.port = port
        self.reader = FrameReader()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        try:
            self.ser.dtr = True
        except Exception:
            pass
        self._frames = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting))
            except Exception:
                return
            for frame in self.reader.feed(chunk):
                self._frames.put(frame)

    def send(self, can_id, payload):
        self.ser.write(encode_frame(can_id, payload))
        self.ser.flush()

    def recv(self):
        frames = []
        while True:
            try:
                frames.append(self._frames.get_nowait())
            except queue.Empty:
                return frames

    @property
    def frames_dropped(self):
        return self.reader.dropped

    def close(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass
        self._thread.join(timeout=1)
