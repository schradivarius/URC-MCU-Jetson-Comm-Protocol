#ifndef ROVER_LINK_CAN
#define ROVER_LINK_CAN 0
#endif

#include <Arduino.h>

#if ROVER_LINK_CAN
#include <FlexCAN_T4.h>
#endif
#include <Watchdog_t4.h>

#include "src/rover_controller.h"
#include "src/rover_drive.h"
#include "src/rover_motor.h"
#include "src/rover_protocol.h"
#include "src/rover_serial.h"

using namespace rover;

static const int LED_PIN = 13;

static const bool MOTOR_HBRIDGE = true;

static const int PIN_DRIVE_L_PWM = 2;
static const int PIN_DRIVE_R_PWM = 3;
static const int PIN_STEER_PWM   = 4;
static const int PIN_DRIVE_L_DIR = 5;
static const int PIN_DRIVE_R_DIR = 6;
static const int PIN_STEER_DIR   = 7;

static const bool SIMULATE_SENSORS = true;

#if ROVER_LINK_CAN
static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
#else
static SerialFrameReader link_reader;
#endif

static WDT_T4<WDT1> wdt;

static RoverController* ctl = nullptr;
static DriveModule drive_stage;
static uint16_t g_boot_faults = 0;

static uint32_t pwmFrequencyHz() {
    return MOTOR_HBRIDGE ? MOTOR_PWM_FREQUENCY_HZ : SERVO_PWM_FREQUENCY_HZ;
}

static void linkBegin() {
    Serial.begin(SERIAL_BAUD_HZ);
#if ROVER_LINK_CAN
    Can.begin();
    Can.setBaudRate(CAN_BITRATE_HZ);
    Can.setMBFilter(REJECT_ALL);
    Can.setMBFilter(MB0, CAN_ID_CONTROL);
    Can.enableMBInterrupts();
#endif
}

static bool linkReady() {
#if ROVER_LINK_CAN
    return true;
#else
    return static_cast<bool>(Serial);
#endif
}

static void linkPoll() {
#if ROVER_LINK_CAN
    CAN_message_t rx;
    while (Can.read(rx)) {
        ctl->ingestFrame(rx.id, rx.buf, rx.len);
    }
#else
    int pending = Serial.available();
    while (pending-- > 0) {
        const int in = Serial.read();
        if (in < 0) break;
        if (link_reader.feed(static_cast<uint8_t>(in))) {
            ctl->ingestFrame(link_reader.frameId(), link_reader.framePayload(),
                             static_cast<uint8_t>(link_reader.framePayloadLen()));
        }
    }
#endif
}

static void linkSend(uint32_t id, const uint8_t* buf, uint8_t len) {
#if ROVER_LINK_CAN
    CAN_message_t frame;
    frame.id  = id;
    frame.len = len;
    for (uint8_t i = 0; i < len && i < 8; ++i) frame.buf[i] = buf[i];
    Can.write(frame);
#else
    uint8_t encoded[SERIAL_ENCODED_MAX];
    const size_t n = encodeSerialFrame(id, buf, len, encoded, sizeof(encoded));
    if (n == 0) return;
    if (Serial.availableForWrite() < static_cast<int>(n)) return;
    Serial.write(encoded, n);
#endif
}

static int32_t sim_enc_left  = 0;
static int32_t sim_enc_right = 0;
static int16_t sim_steer_fb  = 0;

struct SensorReading {
    int32_t  enc_left;
    int32_t  enc_right;
    int16_t  steer_fb;
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
        r.enc_left = 0; r.enc_right = 0; r.steer_fb = 0;
        r.current_ca = 0; r.voltage_cv = 0; r.fault_bits = 0;
        r.controller_health = drive_stage.health();
        r.drive_applied = 0; r.steer_applied = 0;
        return r;
    }

    const int16_t target = out.steer_effort;

    sim_enc_left  = wrapI32(static_cast<int64_t>(sim_enc_left)  + out.left_effort / 10);
    sim_enc_right = wrapI32(static_cast<int64_t>(sim_enc_right) + out.right_effort / 10);

    if (sim_steer_fb < target)      sim_steer_fb = min(target, (int16_t)(sim_steer_fb + 25));
    else if (sim_steer_fb > target) sim_steer_fb = max(target, (int16_t)(sim_steer_fb - 25));

    const int32_t effort_mag =
        (abs(out.left_effort) + abs(out.right_effort)) / 2;

    r.enc_left   = sim_enc_left;
    r.enc_right  = sim_enc_right;
    r.steer_fb   = sim_steer_fb;
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

static void applyChannel(int pin_pwm, int pin_dir, int16_t effort,
                         uint16_t pulse_us) {
    if (MOTOR_HBRIDGE) {
        const MotorDuty d = hbridgeDuty(effort, PWM_RESOLUTION_BITS);
        digitalWrite(pin_dir, d.reverse ? HIGH : LOW);
        analogWrite(pin_pwm, d.duty);
        return;
    }
    analogWrite(pin_pwm,
                servoPulseDuty(pulse_us, PWM_RESOLUTION_BITS, pwmFrequencyHz()));
}

static void setMotorOutputs(const DriveOutputs& out) {
    applyChannel(PIN_DRIVE_L_PWM, PIN_DRIVE_L_DIR, out.left_effort, out.left_pwm_us);
    applyChannel(PIN_DRIVE_R_PWM, PIN_DRIVE_R_DIR, out.right_effort, out.right_pwm_us);
    applyChannel(PIN_STEER_PWM,   PIN_STEER_DIR,   out.steer_effort, out.steer_pwm_us);
}

static void holdOutputsNeutral() {
    const int dir_pins[] = {PIN_DRIVE_L_DIR, PIN_DRIVE_R_DIR, PIN_STEER_DIR};
    for (int pin : dir_pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    const uint32_t neutral = MOTOR_HBRIDGE
        ? 0u
        : servoPulseDuty(PWM_NEUTRAL_US, PWM_RESOLUTION_BITS, pwmFrequencyHz());
    const int pwm_pins[] = {PIN_DRIVE_L_PWM, PIN_DRIVE_R_PWM, PIN_STEER_PWM};
    for (int pin : pwm_pins) {
        pinMode(pin, OUTPUT);
        analogWriteFrequency(pin, pwmFrequencyHz());
        analogWrite(pin, neutral);
    }
}

static uint16_t detectBootFault() {
    const uint32_t srsr = SRC_SRSR;
    SRC_SRSR = srsr;
    const uint32_t WDOG_BITS = (1u << 4) | (1u << 7);
    return (srsr & WDOG_BITS) ? FAULT_FIRMWARE_FAULT : 0;
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
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    analogWriteResolution(PWM_RESOLUTION_BITS);
    holdOutputsNeutral();

    linkBegin();
    g_boot_faults = detectBootFault();

    static RoverController controller(millis, DEFAULT_WATCHDOG_TIMEOUT_MS,
                                      DEFAULT_TELEMETRY_PERIOD_MS);
    ctl = &controller;

    WDT_timings_t wdt_config;
    wdt_config.trigger = (HW_WATCHDOG_MS / 1000.0) / 2.0;
    wdt_config.timeout = HW_WATCHDOG_MS / 1000.0;
    wdt.begin(wdt_config);

#if ROVER_LINK_CAN
    Serial.printf("rover firmware up: CAN3 @ %lu bps, hw watchdog %lu ms%s\n",
                  (unsigned long)CAN_BITRATE_HZ, (unsigned long)HW_WATCHDOG_MS,
                  g_boot_faults ? ", RECOVERED FROM WATCHDOG RESET" : "");
#endif
}

void loop() {
    wdt.feed();

    linkPoll();

    int16_t drive, steer;
    ctl->commandedOutputs(drive, steer);
    const bool stopped = ctl->effectiveStop();
    const DriveOutputs& out = drive_stage.update(drive, steer, !stopped);

    SensorReading s = readSensors(out);
    s.drive_applied = drive;
    s.steer_applied = steer;
    setMotorOutputs(out);

    if (ctl->telemetryDue()) {
        if (linkReady()) sendTelemetry(s);
        digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    }
}
