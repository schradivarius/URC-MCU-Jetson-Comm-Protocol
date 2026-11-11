// rover_firmware.cpp -- Rover embedded controller firmware.
// Target: Teensy 4.1 (NXP i.MX RT1062, Cortex-M7 @ 600 MHz), Teensyduino core.
//
// BUILD
//   PlatformIO: `pio run` to compile, `pio run -t upload` to flash. See
//   platformio.ini in the repo root; it names the board and both libraries:
//     FlexCAN_T4   https://github.com/tonton81/FlexCAN_T4
//     WDT_T4       https://github.com/tonton81/WDT_T4  (header: Watchdog_t4.h)
//   Sources under src/ are compiled automatically.
//
//   This is a .cpp, not a .ino, so it is a plain translation unit: nothing
//   rewrites it before the compiler sees it, and Arduino.h is included
//   explicitly below rather than injected. That is why setup() and loop()
//   appear at the bottom -- every function above them is defined before it is
//   used, with no generated forward declarations to depend on. Keep it that
//   way: a new helper goes above its first caller.
//
// WIRING -- READ BEFORE POWERING ANYTHING
//   * Teensy 4.1 is 3.3 V and NOT 5 V tolerant. A 5 V encoder or sensor wired
//     direct to a pin destroys the board. Level-shift everything.
//   * CAN needs a TRANSCEIVER per bus; the MCU cannot drive a differential
//     bus itself. CAN3 is the FD-capable bus on Teensy 4.1.
//   * The bus needs 120 ohm termination at BOTH physical ends. Exactly one
//     resistor, or three, is the most common CAN bring-up failure.
//
// WHY THIS FILE IS SHORT
//   All the protocol and safety logic lives in src/rover_protocol.* and
//   src/rover_controller.*, which are pure C++ with no Arduino dependency and
//   are unit-tested natively (`make -C tests/cpp test`). This file
//   is only the hardware wiring around them. If you are reviewing safety
//   behaviour, read rover_controller.h, not this.
//
// SAFETY STRUCTURE
//   1. Hardware watchdog (WDT_T4). If this loop stops feeding it, the Teensy
//      resets, which drives every pin to a safe state and de-energizes the
//      drives. This is the backstop for a hang or a crash.
//   2. Boot-fault reporting. The reset itself is invisible to the Jetson
//      except as a gap in telemetry, so the NEXT boot inspects the reset
//      status register and raises FAULT_FIRMWARE_FAULT for the whole session.
//   3. Command watchdog, in RoverController: no valid CONTROL frame within
//      300 ms forces a stop and raises FAULT_COMM_TIMEOUT.
//   4. Commands are zeroed by commandedOutputs() before they ever reach the
//      motor layer, so a future edit to setMotorOutputs() cannot accidentally
//      act on a stale command. DriveModule then refuses to energize anything
//      when told not to, so the stop survives an edit to either layer.
//   5. Command authority (v0.6). An autonomy-generated command may not drive
//      a teleoperated rover and vice versa; see commandAuthorized(). The mux
//      is here rather than on the Jetson so that a Jetson-side bug cannot
//      drive an autonomous rover from the teleop path.
//
//   Note one honest difference from the CircuitPython version: there is no
//   try/except guard here, because Arduino builds run without exceptions. The
//   hardware watchdog is the whole story for an unexpected fault, which is
//   why arming it is not optional.

#include <Arduino.h>

#include <Watchdog_t4.h>

#include "src/rover_controller.h"
#include "src/rover_drive.h"
#include "src/rover_drive_sim.h"
#include "src/rover_encoder.h"
#include "src/rover_steer_sim.h"
#include "src/rover_feedback_faults.h"
#include "src/rover_protocol.h"
#include "src/rover_serial.h"

using namespace rover;

// ---------------------------------------------------------------------------
// Configuration
//
// Protocol and timing constants (CAN_BITRATE_HZ, HW_WATCHDOG_MS, the
// timeouts, the CAN ids, OVER_CURRENT_CA, UNDERVOLTAGE_CV) live in
// src/rover_config.h. Only board-specific wiring stays here.
// ---------------------------------------------------------------------------

static const int LED_PIN = 13;   // Teensy onboard LED

// Drive-stage output pins. PLACEHOLDERS -- nothing is wired to them yet, and
// they need checking against the Teensy 4.1 PWM-capable pin list and against
// whatever the drivetrain turns out to be. See setMotorOutputs().
static const int PIN_DRIVE_L = 2;
static const int PIN_DRIVE_R = 3;
static const int PIN_STEER   = 4;

// Servo/ESC pulse rate. 50 Hz is the universally safe default; many modern
// ESCs accept far more, which is worth having once the hardware is known,
// because 50 Hz puts up to 20 ms of latency on the end of a 20 Hz loop.
static const int PWM_FREQUENCY_HZ = 50;
static const int PWM_RESOLUTION_BITS = 12;
// LED_PIN is a telemetry heartbeat blink ONLY. It is not the status indicator.
//
// TODO (hardware): the URC status light has no pin assignment and no driver
// yet. RoverController::indicatorState() already decides what it should show
// and TELEM_STATE reports it, so the whole chain exists except the output --
// indicator_state is currently a value the Jetson can read and nobody can see.
//
// This costs POINTS, not just compliance: rule 1.e.xvii scores a target as
// reached only on "autonomously stopping within 1m of the target location and
// indicating its arrival", 25 points per target. So it is its own hardware
// item, not part of the setMotorOutputs() stub.
//
// Rule 1.e.ii also constrains the hardware, not only the colours: the
// indicator must be ON THE BACK of the rover and VISIBLE IN BRIGHT DAYLIGHT
// (it suggests an LED array or high-power LED). Every event runs in full
// daylight, so a bench-sized indicator will not pass.
//
// Mapping (PROTOCOL.md 3.5.1):
//   INDICATOR_OFF        off
//   INDICATOR_TELEOP     blue
//   INDICATOR_AUTONOMOUS red
//   INDICATOR_ARRIVED    flashing green   (this firmware owns the flash rate)
//   INDICATOR_FAULT      yellow, solid    (placeholder -- see PROTOCOL.md 9)
// Flip to false once real encoders / current sensing / steering feedback are
// wired in. See readSensors() for where the real code goes.
static const bool SIMULATE_SENSORS = true;

static SerialFrameReader link_reader;
WDT_T4<WDT1> wdt;

static RoverController* ctl = nullptr;
static DriveModule drive_stage;
static uint16_t g_boot_faults = 0;

// ---------------------------------------------------------------------------
// Sensor / motor I/O -- placeholder implementations.
//
// These are the ONLY functions that should need touching once real hardware
// is available. Everything above is protocol and safety and should not change.
//
// Teensy 4.1 gives you 55 digital I/O and 18 analog inputs, so unlike the
// previous board you can wire encoders and current sense directly. Putting
// the motor controllers on CAN instead is still usually the better call --
// they report their own encoder counts and current over the bus.
// ---------------------------------------------------------------------------

static DriveSim sim_plant;
static SimulatedEncoder sim_enc_l, sim_enc_r;
static SteerSim sim_steer;
static FeedbackFaultInjector sim_faults;

// Task 5's controlled feedback failures, for bench demonstrations. Set
// this to FB_FAULT_ENCODER_FROZEN, FB_FAULT_STEER_FROZEN,
// FB_FAULT_ENCODER_IMPOSSIBLE, FB_FAULT_STEER_IMPOSSIBLE or
// FB_FAULT_DELAYED and reflash to watch the rover report it.
//
// A build-time constant, not a CONTROL field, ON PURPOSE: a way to break
// the rover's own sensors from the Jetson is a weapon, not a feature, and
// nothing on the competition course should be able to reach it. The host
// simulator takes --inject instead, which is where the automated tests
// drive it from.
static const uint8_t SIMULATED_FEEDBACK_FAULT = FB_FAULT_NONE;

struct SensorReading {
    int32_t  enc_left;
    int32_t  enc_right;
    int16_t  steer_fb;
    int16_t  measured_drive;  // mean wheel speed, CMD_MIN..CMD_MAX scale
    uint32_t encoder_update_ms;  // clock at the last encoder change
    uint32_t steer_update_ms;    // clock at the last steering change
    bool     encoder_sane;       // the reading is one the hardware could give
    bool     steer_sane;
    int16_t  current_ca;   // SIGNED centiamps  (1 cA = 10 mA)
    int16_t  voltage_cv;   // SIGNED centivolts (1 cV = 10 mV)
    uint16_t fault_bits;
    uint8_t  controller_health;  // CTRL_HEALTH_*
    int16_t  drive_applied;      // what the drive stage was handed this cycle
    int16_t  steer_applied;
};

// Takes the DRIVE STAGE's output rather than the raw command, so the
// simulated plant reflects the mix the real hardware would receive -- the
// same shape as tools/rover_sim.cpp, and the reason the two agree.
static SensorReading readSensors(const DriveOutputs& out) {
    SensorReading r;
    if (!SIMULATE_SENSORS) {
        // --- real sensor reads go here ---
        //   enc_*       : i.MX RT quadrature encoder peripherals, or CAN
        //   steer_fb    : analogRead() scaled into -1000..1000
        //   measured_drive : mean of the two wheel speeds, scaled into
        //                 -1000..1000 (full scale = speed at CMD_MAX)
        //   current_ca  : analogRead() scaled to SIGNED centiamps (1 cA=10 mA)
        //   voltage_cv  : analogRead() on the pack divider, SIGNED centivolts
        //   fault_bits  : FAULT_OVER_CURRENT / FAULT_ENCODER_FAULT / etc.
        //   controller_health : CTRL_HEALTH_* from motor-driver telemetry,
        //                 which on a CAN bus arrives as its own messages
        r.enc_left = 0; r.enc_right = 0; r.steer_fb = 0; r.measured_drive = 0;
        // No reading yet, so no timestamp to vouch for: the controller
        // reports FEEDBACK_AGE_UNKNOWN until real sensors fill these in.
        r.encoder_update_ms = millis(); r.steer_update_ms = millis();
        r.encoder_sane = true; r.steer_sane = true;
        r.current_ca = 0; r.voltage_cv = 0; r.fault_bits = 0;
        r.controller_health = drive_stage.health();
        r.drive_applied = 0; r.steer_applied = 0;   // filled in by loop()
        return r;
    }

    // wrapI32 takes int64 and wraps explicitly. Letting an int32 counter
    // overflow would be UNDEFINED BEHAVIOUR in C++ -- not a wrap, not an
    // exception, but a compiler free to do anything at all.
    const uint32_t now_ms = millis();
    sim_plant.update(out.left_effort, out.right_effort, out.energized, now_ms);
    sim_enc_l.update(scaledFromMps(sim_plant.left().measured_mps), now_ms);
    sim_enc_r.update(scaledFromMps(sim_plant.right().measured_mps), now_ms);
    // Fed the STEERING STAGE's output, the same place the drive plant is
    // fed from, so a real actuator replaces the module and nothing else.
    sim_steer.update(out.steer_effort, now_ms);

    const int32_t effort_mag =
        (abs(out.left_effort) + abs(out.right_effort)) / 2;

    // THE SENSOR BOUNDARY. Everything below reads ef/sf and nothing else,
    // so an injected fault is wrong EVERYWHERE a real broken sensor would
    // be -- in the echoed telemetry and the measured-vs-commanded feed too,
    // not only in the freshness report. A no-op while
    // SIMULATED_FEEDBACK_FAULT is FB_FAULT_NONE.
    EncoderFeedback ef = sim_enc_l.feedback(now_ms);
    SteerFeedback   sf = sim_steer.feedback(now_ms);
    sim_faults.arm(SIMULATED_FEEDBACK_FAULT);
    sim_faults.apply(ef, sf, now_ms);

    r.enc_left   = ef.count;
    // The right encoder is NOT injected: a wheel sensor dies on its own
    // wheel, and the asymmetry against a healthy right encoder is part of
    // what makes the fault recognisable on a dashboard.
    r.enc_right  = sim_enc_r.feedback(now_ms).count;
    r.steer_fb   = sf.measured;
    // The mean of the two wheels is the drive the rover actually delivered:
    // the mix is left = drive + steer, right = drive - steer.
    r.measured_drive = scaledFromMps(
        (sim_plant.left().measured_mps + sim_plant.right().measured_mps) / 2.0f);

    r.encoder_update_ms = ef.last_update_ms;
    r.steer_update_ms   = sf.last_update_ms;
    // What the plant knows and the controller cannot: the range each of
    // these units can physically take.
    r.encoder_sane = ef.rpm >= -ENCODER_MAX_RPM && ef.rpm <= ENCODER_MAX_RPM;
    r.steer_sane   = isValidCommand(sf.measured);
    r.current_ca = out.energized ? static_cast<int16_t>(effort_mag * 3 / 2) : 0;
    // Fake 24 V pack sagging a little under load.
    r.voltage_cv = static_cast<int16_t>(2400 - effort_mag / 40);
    r.fault_bits = 0;
    if (r.current_ca > OVER_CURRENT_CA) r.fault_bits |= FAULT_OVER_CURRENT;
    if (r.voltage_cv < UNDERVOLTAGE_CV) r.fault_bits |= FAULT_UNDERVOLTAGE;
    // The drive stage reports its own health now. Still not real ESC
    // telemetry -- that arrives as its own CAN messages and overrides this --
    // but no longer a constant NOT_REPORTED.
    r.controller_health = drive_stage.health();
    r.drive_applied = 0;                        // filled in by loop()
    r.steer_applied = 0;
    return r;
}

// Convert a pulse width to the duty value analogWrite() wants at the
// configured frequency and resolution. Kept here rather than in rover_drive.*
// because it is board configuration, not protocol: the module speaks
// microseconds, which every servo and ESC datasheet also speaks.
static uint32_t pulseToDuty(uint16_t pulse_us) {
    const uint32_t full_scale = (1u << PWM_RESOLUTION_BITS) - 1u;
    const uint32_t period_us = 1000000u / PWM_FREQUENCY_HZ;
    return (static_cast<uint32_t>(pulse_us) * full_scale) / period_us;
}

static void setMotorOutputs(const DriveOutputs& out) {
    // The module has already applied the stop: every channel reads neutral
    // when it was told not to energize, so there is no second copy of the
    // rule here and no way for this function to disagree with it.
    //
    // VERIFY ON HARDWARE. analogWrite() produces a continuous PWM waveform,
    // which is what a brushed H-bridge or a PWM-input ESC wants. A hobby servo
    // or a PPM-input ESC wants a 1-2 ms pulse in a 20 ms frame instead, which
    // is the same numbers through a servo library rather than this. Check
    // which the drive hardware expects BEFORE connecting anything that moves.
    analogWrite(PIN_DRIVE_L, pulseToDuty(out.left_pwm_us));
    analogWrite(PIN_DRIVE_R, pulseToDuty(out.right_pwm_us));
    analogWrite(PIN_STEER,   pulseToDuty(out.steer_pwm_us));
}

// ---------------------------------------------------------------------------
// Boot fault detection
// ---------------------------------------------------------------------------

static uint16_t detectBootFault() {
    // SRC_SRSR is the i.MX RT reset status register; a watchdog reset means
    // the previous run of this firmware stopped feeding the watchdog, i.e. it
    // hung or crashed. The Jetson needs to know: telemetry resuming after an
    // unexplained gap otherwise looks like a flaky bus rather than a
    // controller that restarted mid-drive.
    //
    // VERIFY ON HARDWARE before trusting this bit mask. It is written to fail
    // SAFE -- an unrecognised reset cause reports nothing rather than raising
    // a false alarm -- but a missed watchdog reset is a silent gap, so confirm
    // it by deliberately hanging the loop once on the bench.
    const uint32_t srsr = SRC_SRSR;
    SRC_SRSR = srsr;                        // write-1-to-clear for next boot
    const uint32_t WDOG_BITS = (1u << 4) | (1u << 7);   // WDOG and WDOG3
    return (srsr & WDOG_BITS) ? FAULT_FIRMWARE_FAULT : 0;
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

static void linkPoll() {
    int pending = Serial.available();
    while (pending-- > 0) {
        const int in = Serial.read();
        if (in < 0) break;
        if (link_reader.feed(static_cast<uint8_t>(in))) {
            ctl->ingestFrame(link_reader.frameId(), link_reader.framePayload(),
                             static_cast<uint8_t>(link_reader.framePayloadLen()));
        }
    }
}

static void linkSend(uint32_t id, const uint8_t* buf, uint8_t len) {
    uint8_t encoded[SERIAL_ENCODED_MAX];
    const size_t n = encodeSerialFrame(id, buf, len, encoded, sizeof(encoded));
    if (n == 0) return;
    if (Serial.availableForWrite() < static_cast<int>(n)) return;
    Serial.write(encoded, n);
}

static void sendTelemetry(const SensorReading& s) {
    const uint8_t seq = ctl->telemetrySeq();
    uint8_t buf[FRAME_DLC];

    TelemetryDriveL dl = ctl->buildDriveL(s.enc_left, s.steer_fb);
    linkSend(CAN_ID_TELEM_DRIVE_L, buf, encodeTelemetryDriveL(dl, seq, buf));

    TelemetryDriveR dr = ctl->buildDriveR(s.enc_right);
    linkSend(CAN_ID_TELEM_DRIVE_R, buf, encodeTelemetryDriveR(dr, seq, buf));

    TelemetryPower pw =
        ctl->buildPower(s.current_ca, s.voltage_cv, s.fault_bits, g_boot_faults);
    linkSend(CAN_ID_TELEM_POWER, buf, encodeTelemetryPower(pw, seq, buf));

    TelemetryState st = ctl->buildState(s.controller_health);
    linkSend(CAN_ID_TELEM_STATE, buf, encodeTelemetryState(st, seq, buf));

    TelemetryCmdEcho ce = ctl->buildCmdEcho(s.drive_applied, s.steer_applied);
    linkSend(CAN_ID_TELEM_CMD_ECHO, buf, encodeTelemetryCmdEcho(ce, seq, buf));

    TelemetryFeedback fb = ctl->buildFeedback();
    linkSend(CAN_ID_TELEM_FEEDBACK, buf, encodeTelemetryFeedback(fb, seq, buf));
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Bring the drive pins up at neutral BEFORE anything else can run. A pin
    // left floating through setup() is a pin an ESC may read as a command.
    analogWriteResolution(PWM_RESOLUTION_BITS);
    for (int pin : {PIN_DRIVE_L, PIN_DRIVE_R, PIN_STEER}) {
        pinMode(pin, OUTPUT);
        analogWriteFrequency(pin, PWM_FREQUENCY_HZ);
        analogWrite(pin, pulseToDuty(PWM_NEUTRAL_US));
    }

    Serial.begin(SERIAL_BAUD_HZ);
    g_boot_faults = detectBootFault();

    static RoverController controller(millis, DEFAULT_WATCHDOG_TIMEOUT_MS,
                                      DEFAULT_TELEMETRY_PERIOD_MS);
    ctl = &controller;

    WDT_timings_t wdt_config;
    // Both fields are doubles in SECONDS. `trigger` defaults to 5 s, which
    // would sit ABOVE our 1 s timeout, so set it explicitly: it is when the
    // optional warning callback fires and must be below the reset timeout.
    wdt_config.trigger = (HW_WATCHDOG_MS / 1000.0) / 2.0;
    wdt_config.timeout = HW_WATCHDOG_MS / 1000.0;
    wdt.begin(wdt_config);
}

void loop() {
    wdt.feed();

    linkPoll();

    // The Task 4 chain, in order: the frame has been validated and the mode
    // and authority checked above; the controller now says what may be
    // driven, and the drive stage turns that into pin outputs.
    int16_t drive, steer;
    ctl->commandedOutputs(drive, steer);        // zeroed under stop OR under
                                                // a command-authority mismatch
    const bool stopped = ctl->effectiveStop();
    const DriveOutputs& out = drive_stage.update(drive, steer, !stopped);

    SensorReading s = readSensors(out);
    ctl->updateFaults(s.fault_bits, s.controller_health);
    ctl->updateMeasured(s.measured_drive, s.steer_fb);
    ctl->updateFeedback(s.encoder_update_ms, s.encoder_sane,
                        s.steer_update_ms,   s.steer_sane);
    // Not sensor readings, so they are filled in here rather than inside
    // readSensors(): they are what the controller AUTHORISED, which is what
    // the acknowledgement frame is about.
    s.drive_applied = drive;
    s.steer_applied = steer;
    setMotorOutputs(out);

    if (ctl->telemetryDue()) {
        if (Serial) sendTelemetry(s);
        digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    }
}
