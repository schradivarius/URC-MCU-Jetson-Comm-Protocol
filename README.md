# URC Rover — Jetson ↔ Teensy Link (Tasks 3 & 4)

Communication link between the rover's **onboard computer** (Jetson Orin Nano,
candidate) and its **embedded controller** (**Teensy 4.1**, C++ / Teensyduino),
over **CAN** — and the autonomy-to-embedded interface built on top of it.

Framed, prioritised CAN messages with watchdog-based fail-safes. Full
specification: **[PROTOCOL.md](PROTOCOL.md)**.

**Task 3** proved a Jetson command reaches the controller intact. **Task 4**
proves a high-level autonomy decision travels that interface, reaches the
drive stage, and that the Jetson can read back that it did — including that
autonomy and teleop commands are refused in each other's modes.

## Status

| | |
|---|---|
| Protocol, safety logic, firmware | Implemented, **280 tests** passing |
| Cross-language codec | Pinned byte-for-byte, plus every validation predicate |
| Autonomy → drive-stage loop (Task 4) | Closed and verified end to end (`make autonomy`) |
| USB-serial (UART) link to a laptop | Implemented, framing pinned both languages |
| **Validated on real hardware** | **Not yet — see [Known gaps](#known-gaps)** |

---

## Quick start

**Nothing to install.** Needs a C++17 compiler and Python 3.9+, both of which
you already have. The host tests compile the *real firmware sources* rather
than mocking them, so there is no dependency to manage.

```bash
git clone https://github.com/schradivarius/URC-MCU-Jetson-Comm-Protocol.git
cd URC-MCU-Jetson-Comm-Protocol

make test      # 160 C++ tests + 120 host tests
make demo      # no-hardware message-exchange demonstration (Task 3)
make autonomy  # mock autonomy stack driving the firmware (Task 4)
```

If all three succeed, your environment is good.

---

## Layout

Two languages, because the two ends of the link are different machines:

```
firmware/                    C++ — runs on the Teensy 4.1
  rover_firmware.cpp           hardware wiring ONLY (link, watchdog, pins)
  src/rover_config.h           every protocol + timing constant ─┐
  src/rover_protocol.*         message definitions + codec       │ pure C++,
  src/rover_controller.*       command/safety state machine      │ no Arduino
  src/rover_drive.*            drive/steering output stage       │
  src/rover_serial.*           SLIP framing for the UART link    │
  src/rover_motor.*            effort -> PWM duty / direction   ─┘

host/                        Python — runs on the Jetson
  rover_config.py              constants (pinned to rover_config.h)
  rover_protocol.py            codec (pinned to the C++ one)
  protocol_root.py             which checkout firmware/src comes from
  can_link.py                  sim + python-can backends
  serial_link.py               USB-serial/UART backend (pinned to rover_serial.*)
  uart_drive.py                send a move/stop over USB serial and watch it
  c2_link.py                   C2 (base-station) link-loss detection
  jetson_test.py               live harness
  demo.py                      the Task 3 demonstration
  autonomy_sim.py              mock autonomy stack — the Task 4 mission

tools/
  rover_sim.cpp                simulator: REAL controller + drive stage, fake plant
  golden_vectors.cpp           emits vectors for cross-language pinning

tests/cpp/                   160 tests — firmware core
tests/host/                  120 tests — config sync, golden vectors, framing,
                                         integration, the autonomy mission
```

The files under `firmware/src/` have **no Arduino dependency and no hardware
calls**, which is what lets the safety-critical logic be unit-tested natively
with a fake clock. `rover_firmware.cpp` is only the wiring around them. **If
you are reviewing safety behaviour, read `rover_controller.h`.**

The autonomy program can also live in a repository of its own — it is a
consumer of this protocol, not part of it. Two files move, the Python codec
stays, and `ROVER_PROTOCOL_ROOT` points the Jetson code at a checkout of this
one. [PROTOCOL.md §10.2](PROTOCOL.md) says why copying the codec instead would
defeat the cross-language pinning.

---

## Running the tests

```bash
make test        # everything
make test-cpp    # firmware core only — no Python needed
make test-host   # host side only — compiles the C++ core
```

| Suite | Tests | Answers |
|---|---:|---|
| `tests/cpp/test_protocol.cpp` | 41 | Wire format, CRC-8, and every validation predicate |
| `tests/cpp/test_controller.cpp` | 68 | **Does the rover stop when it should?** And: does it refuse a command from the wrong half of the stack? |
| `tests/cpp/test_drive.cpp` | 17 | Does a command become the right motor output — and never one it was told not to? |
| `tests/cpp/test_motor.cpp` | 17 | Does an effort become the right PWM duty and direction, and does a stop reach both output modes? |
| `tests/cpp/test_serial.cpp` | 17 | Does a frame survive the UART link — and does noise, a reset mid-frame or a truncated frame cost exactly one? |
| `tests/host/test_config_sync.py` | 14 | Do the C++ and Python configs hold the same constants? |
| `tests/host/test_serial_framing.py` | 22 | Do the two UART framers agree byte-for-byte, and does framing compose with the CRC? |
| `tests/host/test_golden_vectors.py` | 30 | Do C++ and Python agree byte-for-byte? |
| `tests/host/test_integration.py` | 44 | Do both ends actually talk to each other? |
| `tests/host/test_autonomy_mission.py` | 10 | Does an autonomy decision reach the drive stage — and would we notice if it stopped? |

Every test is named for the failure it prevents, so the reason it exists
outlives anyone's memory of writing it. **Start a safety review at
`test_controller.cpp`.**

Three of these were verified to *fail* against a deliberately broken
implementation, rather than merely assumed to work:

- `watchdog_is_correct_across_millis_wraparound` — fails against a naive
  timestamp comparison
- `test_python_encoder_matches_cpp_byte_for_byte` — fails against an injected
  endianness flip
- `wrong_dlc_does_not_refresh_the_watchdog` — the version-mismatch guard
- `a_bad_crc_does_not_refresh_the_watchdog` — the application-layer CRC's whole
  purpose: corruption in the software path, which CAN's own CRC cannot see
- `the_c2_link_is_never_derived_from_the_jetson_link` and
  `c2_loss_stops_a_teleoperated_rover_but_not_an_autonomous_one` — the 2027
  course has areas with no C2 line-of-sight while autonomy keeps working;
  collapsing the two links would stop the rover exactly where it should keep
  going
- `a_stale_c2_bit_is_reported_as_unknown_not_as_ok_or_lost` — once the Jetson
  goes silent the last forwarded C2 verdict is worthless, so the controller
  stops asserting it rather than latching a claim it cannot vouch for
- `a_non_stopping_fault_does_not_blank_the_indicator` — URC requires red while
  autonomous, so an undervoltage warning must not take the light away from
  what the rover is actually doing
- `undefined_mode_does_not_permit_motion` — fails against the pre-fix
  `mode == MODE_DISABLED` check (that mode is `MODE_SAFE` now), which permitted full throttle on any
  undefined mode ([issue #4](https://github.com/schradivarius/URC-MCU-Jetson-Comm-Protocol/issues/4))
- `the_same_command_is_accepted_once_the_mode_matches` — one command, refused
  in `TELEOP` and applied in `AUTONOMOUS`, with nothing else changed: the Task
  4 claim that manual and autonomous commands do not get mixed up
- `test_an_impossible_expectation_fails_the_run` and
  `test_every_step_passed_on_telemetry_it_caused` — the negative controls for
  the autonomy mission. A harness that cannot fail is not evidence, and a step
  that passes on the *previous* step's telemetry is the classic way an
  integration test goes green while proving nothing

CI additionally builds with **clang** as a second toolchain and runs the suite
under **UBSan + ASan**, because signed overflow in the encoder accumulators
would be undefined behaviour rather than a wrap.

---

## Running against hardware

### The Teensy over USB serial — the bring-up path

The default build carries the protocol over the Teensy's **native USB serial
port**, so a laptop and a USB cable are the whole setup. No CAN transceiver, no
USB-CAN adapter, no Linux. This is what to use to prove the MCU receives a
command and acts on it.

**1. Flash it.** In VS Code with the PlatformIO extension, the status bar along
the bottom has ✓ (Build), → (Upload) and the plug (Serial Monitor); the
PlatformIO sidebar has the same under *Project Tasks → teensy41 → General*. On
the command line it is:

```bash
pio run              # compile the USB-serial build (the default environment)
pio run -t upload     # compile and flash, board plugged in over USB
```

The Teensy loader may ask you to press the button on the board the first time.

**2. Drive it.** `host/uart_drive.py` is the laptop end. It works on Windows,
macOS and Linux and needs only pyserial:

```bash
python -m pip install pyserial
python host/uart_drive.py --list-ports                        # find the port
python host/uart_drive.py --port COM5                         # watch only
python host/uart_drive.py --port COM5 --drive 300 --seconds 3  # move
python host/uart_drive.py --port COM5 --drive 300 --steer -200 --seconds 5
python host/uart_drive.py --port COM5 --stop
```

`--drive` and `--steer` are tenths of a percent, `-1000..1000`. On Linux or
macOS the port looks like `/dev/ttyACM0` or `/dev/cu.usbmodem101`.

The script prints the MCU's own account of what it did with each command —
mode, command age, the drive and steer values it actually applied, and why if
it applied none:

```
run   mode=TELEOP cmd_age=4ms  applied= +300/   +0 status=APPLIED ...
```

`APPLIED` with a non-zero value is the thing to look for: the MCU received the
frame, validated it, decided it was allowed to act on it, and drove the output
stage. It exits non-zero and says which if that never happened.

**It sends continuously, on purpose.** The firmware stops the drives if no
valid command arrives within 300 ms, so a one-shot "move" command is not on
offer — that would be a rover that keeps driving after the laptop is
unplugged. Every exit path, Ctrl-C included, sends stop frames on the way out.

**The USB port is a binary channel in this build.** There is no boot banner,
because text mixed into framed binary is how a bring-up loses an afternoon.
`pio device monitor` will show gibberish, which is itself proof of life. The
pin 13 LED toggles once per telemetry period whether or not a host is
listening.

> **This is USB serial, not a hardware UART.** The Teensy's native USB port is
> what `--port` opens. If you instead want pins 0/1 (`Serial1`) behind a
> USB-TTL adapter, that is a two-line change in `linkBegin()`/`linkPoll()`/
> `linkSend()` plus a 3.3 V adapter — ask, don't improvise, because a 5 V
> adapter on those pins destroys the board.

### Wiring motors

The drive pins and the output mode are the top of
`firmware/rover_firmware.cpp`, deliberately together and deliberately short:

| Constant | Default | What it is |
|---|---|---|
| `MOTOR_HBRIDGE` | `true` | `true` = PWM + direction pin (brushed H-bridge). `false` = servo/ESC pulse, 1500 µs neutral. |
| `PIN_DRIVE_L_PWM` / `_DIR` | 2 / 5 | left channel |
| `PIN_DRIVE_R_PWM` / `_DIR` | 3 / 6 | right channel |
| `PIN_STEER_PWM` / `_DIR` | 4 / 7 | steering actuator |

**One direction pin per channel**, which suits a BTS7960, a Cytron MD-series or
anything else with a `PWM` + `DIR` input pair. An **L298N** is different: it
wants `ENA` plus *two* logic pins (`IN1`, `IN2`) per motor, so it needs a second
pin per channel driven as the inverse. That is a three-line change in
`applyChannel()` — say so before wiring one rather than hoping one pin is
enough.

Frequencies, resolution and the deadband are in `firmware/src/rover_config.h`
(`MOTOR_PWM_FREQUENCY_HZ` 10 kHz, `SERVO_PWM_FREQUENCY_HZ` 50 Hz,
`PWM_RESOLUTION_BITS` 12, `MOTOR_DEADBAND_EFFORT` 20).

**`MOTOR_HBRIDGE` defaults to `true` because that is the direction in which a
wrong guess is harmless.** An H-bridge is what a brushed motor on a driver
board (L298N, BTS7960, Cytron MD-series) wants: duty 0 at rest, so a wrong
guess leaves the motor still. The servo/ESC mode idles at a 1500 µs pulse,
which an H-bridge would read as 7.5% duty and *turn the motor*. Check which
your driver expects before connecting anything that moves, and flip the one
constant if it is an ESC.

> **Before powering anything.**
> - Teensy 4.1 is **3.3 V and NOT 5 V tolerant.** A 5 V sensor or a 5 V logic
>   output on a pin destroys the board. Level-shift everything.
> - **Never** power motors from the Teensy. Separate supply, common ground,
>   and nothing back-feeding the 3.3 V rail.
> - First flash with the drive pins **disconnected**. The LED blink and
>   `uart_drive.py` reporting `APPLIED` prove the whole chain with nothing able
>   to move. Then wire one channel, wheels off the ground.
> - There is **no brake-before-reverse delay**. A command reversal flips the
>   direction pin in the same cycle the duty changes. A sign-magnitude driver
>   handles that; check yours, and keep early commands small.

### A virtual CAN bus (no hardware, real CAN stack)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
pip install python-can
python3 host/jetson_test.py --channel vcan0
```

### The real Teensy over CAN

The CAN build is the same firmware with the transport swapped, kept so the
UART work is not a fork:

```bash
pio run -e teensy41_can
pio run -e teensy41_can -t upload
pio device monitor   # the boot banner, 115200 — this build does print one
```

Then, from the Jetson side:

```bash
python3 host/jetson_test.py --channel can0 --bitrate 500000
```

Everything above the transport — framing, the CRC, sequence numbers, the
command watchdog, authority, the drive stage — is byte-identical between the
two environments. Only `linkBegin()`, `linkPoll()`, `linkReady()` and
`linkSend()` differ, under `ROVER_LINK_CAN`.

> **Why not the Arduino IDE.** The entry point is `firmware/rover_firmware.cpp`,
> a plain translation unit rather than an Arduino `.ino` sketch. The IDE needs a
> `.ino` to open a sketch at all, so it cannot build this tree as it stands. The
> gain is that nothing rewrites the file before the compiler sees it: a sketch
> gets forward declarations injected near the top, which land ahead of the
> structs they name in a file shaped like this one. If you do want the IDE back,
> add a `firmware/rover_firmware.ino` containing the single line
> `#include "rover_firmware.cpp"` — but then keep both toolchains honest, because
> only one of them is exercised here.

> **Before powering a CAN bus.**
> - Each CAN bus needs a **transceiver** — the MCU cannot drive a differential
>   bus directly.
> - The bus needs **120 Ω termination at both physical ends.** Exactly one
>   resistor, or three, is the single most common CAN bring-up failure.

The CAN build prints a banner over USB serial on boot:

```
rover firmware up: CAN3 @ 500000 bps, hw watchdog 1000 ms
```

`RECOVERED FROM WATCHDOG RESET` appended means the previous run hung or
crashed — see `PROTOCOL.md` §5.2.

---

## Known gaps

Stated plainly, so nobody builds on an unchecked assumption:

- **Never run on a real Teensy, a real CAN bus or a real serial port.** Every
  library call was verified against the `FlexCAN_T4` and `WDT_T4` headers, and
  the whole firmware — `rover_firmware.cpp` plus everything in `src/` —
  compiles *and links* under both g++ and clang at `-Wall -Wextra -Werror`
  against stub `Arduino.h`, `FlexCAN_T4.h` and `Watchdog_t4.h`, in **both** the
  `ROVER_LINK_CAN=0` and `=1` configurations. What that does not cover is the
  real `arm-none-eabi` build against the actual Teensyduino and library
  headers, which has never run, and the board, which has never been flashed.
  Bench validation is owed.
- **The UART link has never carried a byte between two machines.** Both framers
  are pinned to the same literal wire bytes
  (`tests/cpp/test_serial.cpp` and `tests/host/test_serial_framing.py`) and
  each is tested against noise, truncation and corruption, but no test runs
  `host/serial_link.py` against the firmware over a port. The firmware side of
  `linkPoll()`/`linkSend()` is exercised only by compiling and linking.
- **`SRC_SRSR` watchdog bit mask is unverified.** It fails safe — an
  unrecognised reset cause reports nothing — but confirm it by deliberately
  hanging the loop once on the bench.
- **Sensor I/O is stubbed** behind `readSensors()`, so every telemetry reading
  except the command echo is synthetic: encoder counts, current and voltage are
  a model, not a measurement. Treat `APPLIED` in `uart_drive.py` as the real
  signal and the encoder counts as decoration until encoders are wired.
- **The drive pins and the output mode are placeholders.** The drive stage
  itself is real and unit-tested (`rover_drive.*`, `rover_motor.*`), but pins
  2–7 are a guess and `MOTOR_HBRIDGE` is a guess at the hardware. It defaults
  to the H-bridge mode because that is the direction in which a wrong guess
  leaves the motor still; see *Wiring motors* above. **Establish which the
  hardware expects before connecting anything that moves.**
- **`return_request` is acknowledged, not acted on.** The controller reports
  that it arrived; whether it should *do* anything is an open team decision in
  `PROTOCOL.md` §9, because it changes `effectiveStop()`.
- **Two encoders, one aggregate current reading.** Both need to grow, and both
  change the payload when they do.
- **CAN FD, the second and third CAN buses, and a dedicated e-stop frame** are
  all unused. The `0x000`–`0x0FF` id block is reserved for that e-stop.

## CI

Every push runs the C++ suite under g++ **and** clang **and** sanitizers, plus
the host suite on Python 3.9 / 3.11 / 3.12 —
[`.github/workflows/ci.yml`](.github/workflows/ci.yml).

## License

GPL-3.0 — see [LICENSE](LICENSE).
