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
