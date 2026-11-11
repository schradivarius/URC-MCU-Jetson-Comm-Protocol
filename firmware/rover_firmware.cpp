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

static const int LED_PIN = 13;

static const int PIN_DRIVE_L = 2;
static const int PIN_DRIVE_R = 3;
static const int PIN_STEER   = 4;

static const int PWM_FREQUENCY_HZ = 50;
static const int PWM_RESOLUTION_BITS = 12;
static const bool SIMULATE_SENSORS = true;

static SerialFrameReader link_reader;
WDT_T4<WDT1> wdt;

static RoverController* ctl = nullptr;
static DriveModule drive_stage;
static uint16_t g_boot_faults = 0;

static DriveSim sim_plant;
static SimulatedEncoder sim_enc_l, sim_enc_r;
static SteerSim sim_steer;
static FeedbackFaultInjector sim_faults;

static const uint8_t SIMULATED_FEEDBACK_FAULT = FB_FAULT_NONE;

struct SensorReading {
    int32_t  enc_left;
    int32_t  enc_right;
    int16_t  steer_fb;
    int16_t  measured_drive;
    uint32_t encoder_update_ms;
    uint32_t steer_update_ms;
    bool     encoder_sane;
    bool     steer_sane;
    int16_t  current_ca;
    int16_t  voltage_cv;
    uint16_t fault_bits;
    uint8_t  controller_health;
    int16_t  drive_applied;
    int16_t  steer_applied;
};

static SensorReading readSensors(const DriveOutputs& out) {
    SensorReading r;
    if (!SIMULATE_SENSORS) {
        r.enc_left = 0; r.enc_right = 0; r.steer_fb = 0; r.measured_drive = 0;
        r.encoder_update_ms = millis(); r.steer_update_ms = millis();
        r.encoder_sane = true; r.steer_sane = true;
        r.current_ca = 0; r.voltage_cv = 0; r.fault_bits = 0;
        r.controller_health = drive_stage.health();
        r.drive_applied = 0; r.steer_applied = 0;
        return r;
    }

    const uint32_t now_ms = millis();
    sim_plant.update(out.left_effort, out.right_effort, out.energized, now_ms);
    sim_enc_l.update(scaledFromMps(sim_plant.left().measured_mps), now_ms);
    sim_enc_r.update(scaledFromMps(sim_plant.right().measured_mps), now_ms);
    sim_steer.update(out.steer_effort, now_ms);

    const int32_t effort_mag =
        (abs(out.left_effort) + abs(out.right_effort)) / 2;

    EncoderFeedback ef = sim_enc_l.feedback(now_ms);
    SteerFeedback   sf = sim_steer.feedback(now_ms);
    sim_faults.arm(SIMULATED_FEEDBACK_FAULT);
    sim_faults.apply(ef, sf, now_ms);

    r.enc_left   = ef.count;
    r.enc_right  = sim_enc_r.feedback(now_ms).count;
    r.steer_fb   = sf.measured;
    r.measured_drive = scaledFromMps(
        (sim_plant.left().measured_mps + sim_plant.right().measured_mps) / 2.0f);

    r.encoder_update_ms = ef.last_update_ms;
    r.steer_update_ms   = sf.last_update_ms;
    r.encoder_sane = ef.rpm >= -ENCODER_MAX_RPM && ef.rpm <= ENCODER_MAX_RPM;
    r.steer_sane   = isValidCommand(sf.measured);
    r.current_ca = out.energized ? static_cast<int16_t>(effort_mag * 3 / 2) : 0;
    r.voltage_cv = static_cast<int16_t>(2400 - effort_mag / 40);
    r.fault_bits = 0;
    if (r.current_ca > OVER_CURRENT_CA) r.fault_bits |= FAULT_OVER_CURRENT;
    if (r.voltage_cv < UNDERVOLTAGE_CV) r.fault_bits |= FAULT_UNDERVOLTAGE;
    r.controller_health = drive_stage.health();
    r.drive_applied = 0;
    r.steer_applied = 0;
    return r;
}

static uint32_t pulseToDuty(uint16_t pulse_us) {
    const uint32_t full_scale = (1u << PWM_RESOLUTION_BITS) - 1u;
    const uint32_t period_us = 1000000u / PWM_FREQUENCY_HZ;
    return (static_cast<uint32_t>(pulse_us) * full_scale) / period_us;
}

static void setMotorOutputs(const DriveOutputs& out) {
    analogWrite(PIN_DRIVE_L, pulseToDuty(out.left_pwm_us));
    analogWrite(PIN_DRIVE_R, pulseToDuty(out.right_pwm_us));
    analogWrite(PIN_STEER,   pulseToDuty(out.steer_pwm_us));
}

static uint16_t detectBootFault() {
    const uint32_t srsr = SRC_SRSR;
    SRC_SRSR = srsr;
    const uint32_t WDOG_BITS = (1u << 4) | (1u << 7);
    return (srsr & WDOG_BITS) ? FAULT_FIRMWARE_FAULT : 0;
}

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

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

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
    wdt_config.trigger = (HW_WATCHDOG_MS / 1000.0) / 2.0;
    wdt_config.timeout = HW_WATCHDOG_MS / 1000.0;
    wdt.begin(wdt_config);
}

void loop() {
    wdt.feed();

    linkPoll();

    int16_t drive, steer;
    ctl->commandedOutputs(drive, steer);
    const bool stopped = ctl->effectiveStop();
    const DriveOutputs& out = drive_stage.update(drive, steer, !stopped);

    SensorReading s = readSensors(out);
    ctl->updateFaults(s.fault_bits, s.controller_health);
    ctl->updateMeasured(s.measured_drive, s.steer_fb);
    ctl->updateFeedback(s.encoder_update_ms, s.encoder_sane,
                        s.steer_update_ms,   s.steer_sane);
    s.drive_applied = drive;
    s.steer_applied = steer;
    setMotorOutputs(out);

    if (ctl->telemetryDue()) {
        if (Serial) sendTelemetry(s);
        digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    }
}
