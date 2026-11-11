#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <string>
#include <poll.h>
#include <unistd.h>

#include "../firmware/src/rover_controller.h"
#include "../firmware/src/rover_drive.h"
#include "../firmware/src/rover_drive_sim.h"
#include "../firmware/src/rover_encoder.h"
#include "../firmware/src/rover_steer_sim.h"
#include "../firmware/src/rover_feedback_faults.h"

using namespace rover;

static std::chrono::steady_clock::time_point g_start;
static uint32_t realMillis() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now() - g_start).count());
}

static void emitFrame(uint32_t id, const uint8_t* buf, uint8_t len) {
    std::printf("%03x ", id);
    for (uint8_t i = 0; i < len; ++i) std::printf("%02x", buf[i]);
    std::printf("\n");
    std::fflush(stdout);
}

int main(int argc, char** argv) {
    bool verbose = false;
    uint32_t seed = 1;
    uint8_t injected_fault = FB_FAULT_NONE;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-v") == 0) verbose = true;
        if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
        }
        if (std::strcmp(argv[i], "--inject") == 0 && i + 1 < argc) {
            if (!parseFeedbackFault(argv[++i], injected_fault)) {
                std::fprintf(stderr, "rover_sim: unknown --inject %s\n", argv[i]);
                return 2;
            }
        }
    }

    g_start = std::chrono::steady_clock::now();
    RoverController ctl(realMillis, DEFAULT_WATCHDOG_TIMEOUT_MS,
                        DEFAULT_TELEMETRY_PERIOD_MS);
    DriveModule drv;
    DriveSim feedback(seed);
    SimulatedEncoder enc_l, enc_r;
    SteerSim steer_plant;
    FeedbackFaultInjector injector;
    injector.arm(injected_fault);
    std::string pending;
    char chunk[512];

    for (;;) {
        struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
        if (poll(&pfd, 1, 2) > 0 && (pfd.revents & (POLLIN | POLLHUP))) {
            ssize_t n = read(STDIN_FILENO, chunk, sizeof(chunk));
            if (n <= 0) break;
            pending.append(chunk, static_cast<size_t>(n));
            size_t nl;
            while ((nl = pending.find('\n')) != std::string::npos) {
                std::string line = pending.substr(0, nl);
                pending.erase(0, nl + 1);
                unsigned id = 0; char hex[32] = {0};
                if (std::sscanf(line.c_str(), "%x %31s", &id, hex) == 2) {
                    uint8_t rx[8] = {0};
                    size_t len = std::strlen(hex) / 2;
                    if (len > 8) len = 8;
                    for (size_t i = 0; i < len; ++i) {
                        unsigned byte = 0;
                        std::sscanf(hex + i * 2, "%2x", &byte);
                        rx[i] = static_cast<uint8_t>(byte);
                    }
                    ctl.ingestFrame(id, rx, static_cast<uint8_t>(len));
                }
            }
        }

        int16_t drive, steer;
        ctl.commandedOutputs(drive, steer);
        const bool stopped = ctl.effectiveStop();
        const DriveOutputs& out = drv.update(drive, steer, !stopped);

        const uint32_t now_ms = realMillis();
        feedback.update(out.left_effort, out.right_effort, out.energized, now_ms);
        enc_l.update(scaledFromMps(feedback.left().measured_mps), now_ms);
        enc_r.update(scaledFromMps(feedback.right().measured_mps), now_ms);
        steer_plant.update(out.steer_effort, now_ms);

        EncoderFeedback ef = enc_l.feedback(now_ms);
        SteerFeedback   sf = steer_plant.feedback(now_ms);
        injector.apply(ef, sf, now_ms);

        const int32_t enc_left  = ef.count;
        const int32_t enc_right = enc_r.feedback(now_ms).count;
        const int16_t steer_fb  = sf.measured;
        const int32_t effort_mag =
            (std::abs(out.left_effort) + std::abs(out.right_effort)) / 2;
        const int16_t current_ca =
            out.energized ? static_cast<int16_t>(effort_mag * 3 / 2) : 0;
        const int16_t voltage_cv = static_cast<int16_t>(2400 - effort_mag / 40);

        uint16_t hw_faults = 0;
        if (current_ca > OVER_CURRENT_CA) hw_faults |= FAULT_OVER_CURRENT;
        if (voltage_cv < UNDERVOLTAGE_CV) hw_faults |= FAULT_UNDERVOLTAGE;
        ctl.updateFaults(hw_faults, drv.health());
        ctl.updateMeasured(
            scaledFromMps((feedback.left().measured_mps +
                           feedback.right().measured_mps) / 2.0f),
            steer_fb);
        ctl.updateFeedback(
            ef.last_update_ms,
            ef.rpm >= -ENCODER_MAX_RPM && ef.rpm <= ENCODER_MAX_RPM,
            sf.last_update_ms,
            isValidCommand(sf.measured));

        if (ctl.telemetryDue()) {
            const uint8_t seq = ctl.telemetrySeq();
            uint8_t buf[8];

            TelemetryDriveL dl = ctl.buildDriveL(enc_left, steer_fb);
            emitFrame(CAN_ID_TELEM_DRIVE_L, buf, encodeTelemetryDriveL(dl, seq, buf));

            TelemetryDriveR dr = ctl.buildDriveR(enc_right);
            emitFrame(CAN_ID_TELEM_DRIVE_R, buf, encodeTelemetryDriveR(dr, seq, buf));

            TelemetryPower pw = ctl.buildPower(current_ca, voltage_cv, hw_faults);
            emitFrame(CAN_ID_TELEM_POWER, buf, encodeTelemetryPower(pw, seq, buf));

            TelemetryState st = ctl.buildState(drv.health());
            emitFrame(CAN_ID_TELEM_STATE, buf, encodeTelemetryState(st, seq, buf));

            TelemetryCmdEcho ce = ctl.buildCmdEcho(drive, steer);
            emitFrame(CAN_ID_TELEM_CMD_ECHO, buf, encodeTelemetryCmdEcho(ce, seq, buf));

            TelemetryFeedback fb = ctl.buildFeedback();
            emitFrame(CAN_ID_TELEM_FEEDBACK, buf, encodeTelemetryFeedback(fb, seq, buf));

            if (verbose) {
                std::fprintf(stderr,
                             "drive: %-12s ack=%3u cmd=(%+5d,%+5d) "
                             "out=(L%+5d R%+5d S%+5d) pwm=(%4u %4u %4u) %s\n",
                             cmdStatusName(ctl.commandStatus()), ce.ack_seq,
                             drive, steer, out.left_effort, out.right_effort,
                             out.steer_effort, out.left_pwm_us, out.right_pwm_us,
                             out.steer_pwm_us,
                             out.energized ? "ENERGIZED" : "neutral");
                std::fprintf(stderr,
                             "feedback: t=%lums dt=%lums  "
                             "L cmd=%+.2f meas=%+.2f m/s enc=%ld (%+.0f t/s)  "
                             "R cmd=%+.2f meas=%+.2f m/s enc=%ld (%+.0f t/s)\n",
                             (unsigned long)feedback.lastUpdateMs(),
                             (unsigned long)feedback.lastDtMs(),
                             feedback.left().commanded_mps, feedback.left().measured_mps,
                             (long)feedback.left().encoder_ticks, feedback.left().ticks_per_s,
                             feedback.right().commanded_mps, feedback.right().measured_mps,
                             (long)feedback.right().encoder_ticks, feedback.right().ticks_per_s);
            }
        }
    }
    return 0;
}
