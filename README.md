# URC Rover — Jetson ↔ Teensy Link (Tasks 3 & 4)

Communication link between the rover's **onboard computer** (Jetson Orin Nano,
candidate) and its **embedded controller** (**Teensy 4.1**, C++ / Teensyduino),
over **USB serial (UART)** — and the autonomy-to-embedded interface built on top of it.

Framed serial messages with watchdog-based fail-safes. The Teensy talks to the
motor controllers over CAN; that side is not in this repository yet beyond
`demo/`. Full
specification: **[PROTOCOL.md](PROTOCOL.md)**.

**Task 3** proved a Jetson command reaches the controller intact. **Task 4**
proves a high-level autonomy decision travels that interface, reaches the
drive stage, and that the Jetson can read back that it did — including that
autonomy and teleop commands are refused in each other's modes.

## Status

| | |
|---|---|
| Protocol, safety logic, firmware | Implemented, **366 tests** passing |
| Cross-language codec | Pinned byte-for-byte, plus every validation predicate |
| Autonomy → drive-stage loop (Task 4) | Closed and verified end to end (`make autonomy`) |
| **Validated on real hardware** | **Not yet — see [Known gaps](#known-gaps)** |

---

## Quick start

**Nothing to install.** Needs a C++17 compiler and Python 3.9+, both of which
you already have. The host tests compile the *real firmware sources* rather
than mocking them, so there is no dependency to manage.

```bash
git clone https://github.com/schradivarius/URC-MCU-Jetson-Comm-Protocol.git
cd URC-MCU-Jetson-Comm-Protocol

make test      # 231 C++ tests + 135 host tests
make demo      # no-hardware message-exchange demonstration (Task 3)
make autonomy  # mock autonomy stack driving the firmware (Task 4)
```

If all three succeed, your environment is good.

---

## Layout

Two languages, because the two ends of the link are different machines:

```
firmware/                    C++ — runs on the Teensy 4.1
  rover_firmware.cpp           hardware wiring ONLY (serial, watchdog, pins)
  src/rover_config.h           every protocol + timing constant ─┐
  src/rover_protocol.*         message definitions + codec       │
  src/rover_controller.*       command/safety state machine      │ pure C++,
  src/rover_drive.*            drive/steering output stage       │ no Arduino
  src/rover_drive_sim.*        simulated drivetrain (lags)       │
  src/rover_encoder.*          simulated wheel encoders          │
  src/rover_steer_sim.*        simulated steering actuator       │
  src/rover_feedback_faults.*  controlled failures of the above  │
  src/rover_serial.*           SLIP framing for the serial link ─┘

host/                        Python — runs on the Jetson
  rover_config.py              constants (pinned to rover_config.h)
  rover_protocol.py            codec (pinned to the C++ one)
  protocol_root.py             which checkout firmware/src comes from
  sim_link.py                  simulator backend
  serial_link.py               serial backend (pyserial)
  c2_link.py                   C2 (base-station) link-loss detection
  jetson_test.py               live harness
  demo.py                      the Task 3 demonstration
  autonomy_sim.py              mock autonomy stack — the Task 4 mission

tools/
  rover_sim.cpp                simulator: REAL controller + drive stage, fake plant
  golden_vectors.cpp           emits vectors for cross-language pinning

tests/cpp/                   231 tests — firmware core
tests/host/                  135 tests — config sync, golden vectors, integration,
                                        the autonomy mission, feedback failures
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
| `tests/cpp/test_protocol.cpp` | 43 | Wire format, CRC-8, and every validation predicate |
| `tests/cpp/test_controller.cpp` | 106 | **Does the rover stop when it should?** And: does it refuse a command from the wrong half of the stack? Does it report feedback it can no longer trust? |
| `tests/cpp/test_drive.cpp` | 17 | Does a command become the right motor output — and never one it was told not to? |
| `tests/cpp/test_drive_sim.cpp` | 9 | Does the fake drivetrain lag the way a real one does, instead of teleporting? |
| `tests/cpp/test_encoder.cpp` | 9 | Do the fake encoders count, wrap and go stale correctly? |
| `tests/cpp/test_steer_sim.cpp` | 14 | Does simulated steering move *through* its range, and is the error signed the way a controller expects? |
| `tests/cpp/test_feedback_faults.cpp` | 16 | When a sensor freezes, lies or runs late, is each one told from the others? |
| `tests/cpp/test_serial.cpp` | 17 | Does serial framing survive noise, resets and every byte value? |
| `tests/host/test_config_sync.py` | 15 | Do the C++ and Python configs hold the same constants? |
| `tests/host/test_golden_vectors.py` | 33 | Do C++ and Python agree byte-for-byte? |
| `tests/host/test_integration.py` | 46 | Do both ends actually talk to each other? |
| `tests/host/test_autonomy_mission.py` | 10 | Does an autonomy decision reach the drive stage — and would we notice if it stopped? |
| `tests/host/test_serial_framing.py` | 22 | Do the C++ and Python framers produce the same bytes? |
| `tests/host/test_feedback_failures.py` | 9 | Does a broken sensor reach the *Jetson* as the right fault — and does the rover keep driving, as Task 5 says it must for now? |

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
  purpose: corruption in the software path, which no link-level check can see
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

### Breaking the sensors on purpose

The simulator can run with one of its fake sensors broken, which is how the
feedback-fault reporting is demonstrated rather than merely asserted:

```bash
make test-host                                   # builds tests/cpp/build/rover_sim
tests/cpp/build/rover_sim --inject encoder-frozen
```

| `--inject` | What breaks |
|---|---|
| `encoder-frozen` | The left encoder repeats one plausible reading forever |
| `steer-frozen` | Steering feedback stops moving |
| `encoder-impossible` | A wheel RPM an order of magnitude past full scale |
| `steer-impossible` | A steering position the actuator cannot reach |
| `delayed` | Both channels report real readings 250 ms late |

The injection is on the **simulated** side (`firmware/src/rover_feedback_faults.*`)
and sits at the sensor boundary, so the firmware is identical either way and is
told nothing — a fault it can only detect from the readings themselves, which
is the only version of the test worth having. On the Teensy the equivalent is
the `SIMULATED_FEEDBACK_FAULT` constant in `firmware/rover_firmware.cpp`, a
build-time choice on purpose: nothing on the wire can break the rover's own
sensors.

Per Task 5 the rover **keeps driving** through all five — the condition is
detected and reported, and choosing a safe state for it is Task 6.

---

## Running against hardware

### The real Teensy

**PlatformIO.** `platformio.ini` is in the repo root and names the board and
its one library, so there is nothing to install by hand:

```bash
pio run              # compile for the Teensy 4.1
pio run -t upload    # compile and flash
```

Then, from the Jetson side:

```bash
pip install pyserial
python3 host/jetson_test.py --port /dev/ttyACM0     # COM5 or similar on Windows
```

> **Why not the Arduino IDE.** The entry point is `firmware/rover_firmware.cpp`,
> a plain translation unit rather than an Arduino `.ino` sketch. The IDE needs a
> `.ino` to open a sketch at all, so it cannot build this tree as it stands. The
> gain is that nothing rewrites the file before the compiler sees it: a sketch
> gets forward declarations injected near the top, which land ahead of the
> structs they name in a file shaped like this one. If you do want the IDE back,
> add a `firmware/rover_firmware.ino` containing the single line
> `#include "rover_firmware.cpp"` — but then keep both toolchains honest, because
> only one of them is exercised here.

First flash is worth doing with **nothing connected to the drive pins**. The USB
port carries binary frames only, so there is no boot banner; the onboard LED
toggling once per telemetry period is the proof of life.

> **Before powering anything.**
> - Teensy 4.1 is **3.3 V and NOT 5 V tolerant.** A 5 V sensor on a pin
>   destroys the board. Level-shift everything.

`FAULT_FIRMWARE_FAULT` in telemetry means the previous run hung or crashed —
see `PROTOCOL.md` §5.2.

---

## Known gaps

Stated plainly, so nobody builds on an unchecked assumption:

- **Never run on a real Teensy.** The firmware cross-compiles and links with
  `arm-none-eabi-g++` against the real Teensy core at `-Wall -Wextra -Werror`,
  and the laptop side of the serial link has run against the real controller
  over a pty. The board itself has never been flashed. Bench validation is owed.
- **`SRC_SRSR` watchdog bit mask is unverified.** It fails safe — an
  unrecognised reset cause reports nothing — but confirm it by deliberately
  hanging the loop once on the bench.
- **Sensor I/O is stubbed** behind `readSensors()`. The drive stage is real and
  unit-tested (`rover_drive.*`), but its output pins are placeholders and the
  PWM mode is unverified: `analogWrite()` suits a brushed H-bridge or a
  PWM-input ESC, while a hobby servo or a PPM-input ESC wants a 1–2 ms pulse in
  a 20 ms frame. **Establish which the hardware expects before connecting
  anything that moves.**
- **`return_request` is acknowledged, not acted on.** The controller reports
  that it arrived; whether it should *do* anything is an open team decision in
  `PROTOCOL.md` §9, because it changes `effectiveStop()`.
- **Two encoders, one aggregate current reading.** Both need to grow, and both
  change the payload when they do.
- **The frames are still 8 bytes with CAN-style ids**, inherited from the old
  transport. Collapsing them into single serial packets is the next step.

## CI

Every push runs the C++ suite under g++ **and** clang **and** sanitizers, plus
the host suite on Python 3.9 / 3.11 / 3.12 —
[`.github/workflows/ci.yml`](.github/workflows/ci.yml).

## License

GPL-3.0 — see [LICENSE](LICENSE).
