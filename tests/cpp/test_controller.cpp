#include "../../firmware/src/rover_controller.h"
#include "../../firmware/src/rover_drive.h"
#include "../../firmware/src/rover_drive_sim.h"
#include "test_framework.h"

using namespace rover;

static uint32_t g_now = 0;
static uint32_t fakeMillis() { return g_now; }
static void advance(uint32_t ms) { g_now += ms; }

static RoverController freshController(uint32_t start_ms = 0) {
    g_now = start_ms;
    return RoverController(fakeMillis, 300, 50);
}

enum Source { SRC_OWNER = -1, SRC_TELEOP = 0, SRC_AUTONOMY = 1 };

static bool sourceBit(Source src, uint8_t mode) {
    if (src == SRC_OWNER) return mode == MODE_AUTONOMOUS;
    return src == SRC_AUTONOMY;
}

static uint8_t g_seq = 0;
static bool sendFrame(RoverController& ctl, int16_t drive, int16_t steer,
                      uint8_t mode, bool stop, bool abort = false,
                      bool ret = false, uint8_t ind = INDICATOR_OFF,
                      bool c2_lost = false, Source src = SRC_OWNER) {
    ControlMsg m;
    m.drive_cmd = drive; m.steer_cmd = steer; m.mode = mode;
    m.stop = stop; m.autonomy_abort = abort; m.return_request = ret;
    m.c2_lost = c2_lost; m.from_autonomy = sourceBit(src, mode);
    m.indicator_request = ind; m.seq = ++g_seq;
    uint8_t buf[8] = {0};
    return ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(m, buf));
}

// Switch modes the way the Jetson does: the operator hands the mode over at
// neutral, then the new owner sends one neutral command.
// CHANGED: the handover no longer sets STOP. Only the operator can release
// STOP now (Systems), so a STOP in the handover would trap autonomy.
static void grantMode(RoverController& ctl, uint8_t mode, bool c2_lost = false) {
    sendFrame(ctl, 0, 0, mode, false, false, false, INDICATOR_OFF,
              c2_lost, SRC_TELEOP);
    sendFrame(ctl, 0, 0, mode, false, false, false, INDICATOR_OFF,
              c2_lost, SRC_OWNER);
}

static bool sendControl(RoverController& ctl, int16_t drive, int16_t steer,
                        uint8_t mode, bool stop, bool abort = false,
                        bool ret = false, uint8_t ind = INDICATOR_OFF,
                        bool c2_lost = false, Source src = SRC_OWNER) {
    if (modePermitsMotion(mode) && ctl.activeMode() != mode) {
        grantMode(ctl, mode, c2_lost);
    }
    return sendFrame(ctl, drive, steer, mode, stop, abort, ret, ind, c2_lost, src);
}

static bool sendWithSeq(RoverController& ctl, uint8_t seq, int16_t drive = 400) {
    ControlMsg m;
    m.drive_cmd = drive; m.steer_cmd = 0; m.mode = MODE_TELEOP;
    m.stop = false; m.autonomy_abort = false; m.return_request = false;
    m.c2_lost = false; m.from_autonomy = false;
    m.indicator_request = INDICATOR_OFF; m.seq = seq;
    uint8_t buf[8] = {0};
    return ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(m, buf));
}

static void boots_stopped_and_safe() {
    RoverController ctl = freshController();
    CHECK(ctl.effectiveStop());
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), CMD_AGE_UNKNOWN);
    CHECK_EQ(ctl.lastControl().mode, MODE_SAFE);
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_LOST);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

static void valid_control_releases_the_stop() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 100, MODE_TELEOP, false));
    CHECK(!ctl.effectiveStop());
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 500); CHECK_EQ(s, 100);
}

static void watchdog_trips_exactly_at_the_boundary() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    advance(299);
    CHECK(!ctl.watchdogTripped());
    advance(1);
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.jetsonLinkState(), LINK_LOST);
}

static void watchdog_is_correct_across_millis_wraparound() {
    RoverController ctl = freshController(0xFFFFFF00u);
    sendControl(ctl, 500, 0, MODE_TELEOP, false);

    advance(80);
    CHECK(g_now > 0xFFFFFF00u);
    CHECK(!ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), 80);

    advance(219);
    CHECK(g_now < 0xFFFFFF00u);
    CHECK(!ctl.watchdogTripped());
    advance(1);
    CHECK(ctl.watchdogTripped());
}

static void explicit_stop_flag_wins_in_any_mode() {
    const uint8_t modes[] = {MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 1000, 500, mode, true);
        CHECK(ctl.effectiveStop());
        int16_t d, s; ctl.commandedOutputs(d, s);
        CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    }
}

// A4: STOP stays on until a neutral command. A drive command can't turn it off.
static void stop_latches_until_a_neutral_command() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    sendControl(ctl, 500, 0, MODE_TELEOP, true);
    CHECK(ctl.effectiveStop());
    CHECK(sendControl(ctl, 500, 0, MODE_TELEOP, false));
    CHECK(ctl.stopLatched());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0);
    CHECK(sendControl(ctl, 0, 0, MODE_TELEOP, false));
    CHECK(!ctl.stopLatched());
    CHECK(sendControl(ctl, 500, 0, MODE_TELEOP, false));
    ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 500);
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
}

static void disabled_mode_forces_stop_at_full_throttle() {
    RoverController ctl = freshController();
    sendControl(ctl, 1000, 1000, MODE_SAFE, false);
    CHECK(ctl.effectiveStop());
}

static void undefined_mode_never_permits_motion() {
    for (int m = 3; m <= 255; m += 29) {
        RoverController ctl = freshController();
        ControlMsg msg;
        msg.drive_cmd = 1000; msg.steer_cmd = 500;
        msg.mode = static_cast<uint8_t>(m);
        msg.stop = false; msg.autonomy_abort = false; msg.return_request = false;
        msg.indicator_request = INDICATOR_OFF; msg.seq = 1;
        uint8_t buf[8] = {0};
        CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(msg, buf)));
        CHECK(ctl.effectiveStop());
        CHECK_EQ(ctl.lastDecodeResult(), DECODE_BAD_MODE);
    }
}

static void autonomy_abort_stops_an_autonomous_rover() {
    RoverController ctl = freshController();
    sendControl(ctl, 800, 0, MODE_AUTONOMOUS, false, true);
    CHECK(ctl.autonomyAbort());
    CHECK(ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

static void autonomy_abort_does_not_stop_a_teleoperated_rover() {
    RoverController ctl = freshController();
    sendControl(ctl, 800, 0, MODE_TELEOP, false, true);
    CHECK(ctl.autonomyAbort());
    CHECK(!ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 800);
}

static void return_request_is_carried_but_does_not_stop() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, true);
    CHECK(ctl.returnRequest());
    CHECK(!ctl.effectiveStop());
}

static void wrong_dlc_does_not_refresh_the_watchdog() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    advance(290);
    uint8_t buf[8] = {0};
    CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, 4));
    CHECK_EQ(ctl.lastDecodeResult(), DECODE_BAD_DLC);
    advance(10);
    CHECK(ctl.watchdogTripped());
}

static void a_bad_crc_does_not_refresh_the_watchdog() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    advance(290);
    for (int i = 0; i < 5; ++i) {
        ControlMsg m;
        m.drive_cmd = 1000; m.steer_cmd = 0; m.mode = MODE_TELEOP;
        m.stop = false; m.autonomy_abort = false; m.return_request = false;
        m.indicator_request = INDICATOR_OFF; m.seq = static_cast<uint8_t>(i);
        uint8_t buf[8] = {0};
        encodeControl(m, buf);
        buf[0] ^= 0xFF;
        CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, FRAME_DLC));
        CHECK_EQ(ctl.lastDecodeResult(), DECODE_BAD_CRC);
    }
    advance(10);
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.faultWord() & FAULT_CRC_ERROR);
    CHECK_EQ(ctl.crcErrorsSeen(), 5u);
}

static void foreign_can_id_does_not_refresh_the_watchdog() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    advance(290);
    uint8_t buf[8] = {0};
    ControlMsg m;
    m.drive_cmd = 999; m.steer_cmd = 0; m.mode = MODE_TELEOP;
    m.stop = false; m.autonomy_abort = false; m.return_request = false;
    m.indicator_request = INDICATOR_OFF; m.seq = 9;
    encodeControl(m, buf);
    CHECK(!ctl.ingestFrame(0x321, buf, FRAME_DLC));
    advance(10);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.lastControl().drive_cmd, 500);
}

static void out_of_range_command_never_moves_the_rover() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, CMD_MAX + 1, 0, MODE_TELEOP, false));
    int16_t drive = -1, steer = -1;
    ctl.commandedOutputs(drive, steer);
    CHECK_EQ(drive, 0);
    CHECK_EQ(steer, 0);
}

static void out_of_range_command_does_not_refresh_the_watchdog() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 0, MODE_TELEOP, false));
    advance(290);
    CHECK(!ctl.watchdogTripped());
    for (int i = 0; i < 5; ++i) sendControl(ctl, 30000, 0, MODE_TELEOP, false);
    advance(20);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.lastControl().drive_cmd, 500);
}

static void out_of_range_command_is_reported_as_a_protocol_error() {
    RoverController ctl = freshController();
    sendControl(ctl, 0, CMD_MIN - 1, MODE_TELEOP, false);
    CHECK(ctl.faultWord() & FAULT_PROTOCOL_ERROR);
    CHECK(!(ctl.faultWord() & FAULT_CRC_ERROR));
    CHECK(sendControl(ctl, 0, CMD_MIN, MODE_TELEOP, false));
    CHECK(!(ctl.faultWord() & FAULT_PROTOCOL_ERROR));
}

static void a_foreign_id_is_not_a_fault() {
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    uint8_t buf[8] = {0};
    ctl.ingestFrame(0x321, buf, 8);
    CHECK(!(ctl.faultWord() & FAULT_PROTOCOL_ERROR));
    CHECK(!(ctl.faultWord() & FAULT_CRC_ERROR));
}

static void contiguous_sequence_numbers_are_healthy() {
    RoverController ctl = freshController();
    for (uint8_t seq = 1; seq <= 20; ++seq) CHECK(sendWithSeq(ctl, seq));
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
    CHECK_EQ(ctl.seqGapsSeen(), 0u);
    CHECK_EQ(ctl.framesLostEstimate(), 0u);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
}

static void a_sequence_gap_is_detected_and_counted() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 10);
    sendWithSeq(ctl, 14);
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    CHECK_EQ(ctl.seqGapsSeen(), 1u);
    CHECK_EQ(ctl.framesLostEstimate(), 3u);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_DEGRADED);
}

static void the_sequence_wrap_is_not_a_gap() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 254);
    sendWithSeq(ctl, 255);
    sendWithSeq(ctl, 0);
    sendWithSeq(ctl, 1);
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
    CHECK_EQ(ctl.seqGapsSeen(), 0u);
}

static void a_duplicate_sequence_number_is_flagged() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 5);
    sendWithSeq(ctl, 5);
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    CHECK_EQ(ctl.framesLostEstimate(), 0u);
}

static void a_sequence_gap_clears_when_the_stream_recovers() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 1);
    sendWithSeq(ctl, 9);
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    sendWithSeq(ctl, 10);
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
}

static void the_first_frame_is_never_a_gap() {
    RoverController ctl = freshController();
    CHECK(sendWithSeq(ctl, 200));
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
    CHECK_EQ(ctl.seqGapsSeen(), 0u);
}

static void the_link_returns_to_ok_after_the_degraded_hold_expires() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 1);
    sendWithSeq(ctl, 5);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_DEGRADED);
    for (uint32_t t = 0; t < LINK_DEGRADED_HOLD_MS + 100; t += 50) {
        advance(50);
        sendWithSeq(ctl, static_cast<uint8_t>(6 + t / 50));
    }
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
}

static void the_c2_link_is_never_derived_from_the_jetson_link() {
    RoverController ctl = freshController();

    CHECK_EQ(ctl.c2LinkState(), LINK_NOT_REPORTED);

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, false);
    TelemetryState st = ctl.buildState();
    CHECK_EQ(st.jetson_link, LINK_OK);
    CHECK_EQ(st.c2_link, LINK_OK);

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, true);
    st = ctl.buildState();
    CHECK_EQ(st.c2_link, LINK_LOST);
    CHECK_EQ(st.jetson_link, LINK_OK);
    CHECK(!ctl.effectiveStop());
}

static void c2_loss_stops_a_teleoperated_rover_but_not_an_autonomous_one() {
    RoverController ctl = freshController();

    sendControl(ctl, 800, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, true);
    CHECK(!ctl.effectiveStop());
    int16_t drive = -1, steer = -1;
    ctl.commandedOutputs(drive, steer);
    CHECK_EQ(drive, 800);

    RoverController tele = freshController();
    sendControl(tele, 800, 0, MODE_TELEOP, false, false, false, INDICATOR_TELEOP);
    sendControl(tele, 800, 0, MODE_TELEOP, false, false, false,
                INDICATOR_TELEOP, true);
    CHECK(tele.effectiveStop());
    tele.commandedOutputs(drive, steer);
    CHECK_EQ(drive, 0);

    sendControl(tele, 800, 0, MODE_TELEOP, false, false, false,
                INDICATOR_TELEOP, false);
    CHECK(tele.effectiveStop());
    CHECK_EQ(tele.activeMode(), MODE_FAULT);
}

static void the_c2_fault_is_raised_in_every_mode() {
    const uint8_t modes[] = {MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 0, 0, mode, false, false, false, INDICATOR_OFF,
                    true);
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) != 0);
        sendControl(ctl, 0, 0, mode, false, false, false, INDICATOR_OFF,
                    false);
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) == 0);
    }
}

static void a_stale_c2_bit_is_reported_as_unknown_not_as_ok_or_lost() {
    for (int lost = 0; lost <= 1; ++lost) {
        RoverController ctl = freshController();
        sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                    INDICATOR_AUTONOMOUS, lost != 0);
        CHECK_EQ(ctl.c2LinkState(), lost ? LINK_LOST : LINK_OK);

        advance(400);
        CHECK_EQ(ctl.c2LinkState(), LINK_NOT_REPORTED);
        CHECK_EQ(ctl.buildState().c2_link, LINK_NOT_REPORTED);
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) == 0);
        CHECK((ctl.faultWord() & FAULT_COMM_TIMEOUT) != 0);
    }
}

static void state_echoes_the_mode_the_controller_believes_in() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false);
    CHECK_EQ(ctl.buildState().mode, MODE_AUTONOMOUS);
    sendControl(ctl, 0, 0, MODE_SAFE, false);
    CHECK_EQ(ctl.buildState().mode, MODE_SAFE);
}

static void active_mode_follows_the_commanded_mode() {
    const uint8_t modes[] = {MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, mode, false);
        CHECK_EQ(ctl.activeMode(), mode);
    }
}

static void comm_timeout_faults_a_moving_rover() {
    const uint8_t modes[] = {MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, mode, false);
        advance(DEFAULT_WATCHDOG_TIMEOUT_MS - 1);
        CHECK_EQ(ctl.activeMode(), mode);
        advance(1);
        CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    }
}

static void comm_timeout_in_safe_is_reported_but_is_not_a_fault() {
    RoverController ctl = freshController();
    sendControl(ctl, 0, 0, MODE_SAFE, false);
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_LOST);
    CHECK(ctl.effectiveStop());
}

static void fault_latches_until_safe_is_commanded() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK(!sendFrame(ctl, 500, 0, MODE_TELEOP, false));
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK(ctl.modeRefused());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);

    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false));
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK_EQ(ctl.latchedFaults(), 0);
    CHECK(!(ctl.faultWord() & FAULT_COMM_TIMEOUT));

    CHECK(sendFrame(ctl, 500, 0, MODE_TELEOP, false));
    ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 500);
}

static void fault_cannot_be_left_straight_into_motion() {
    const uint8_t modes[] = {MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, MODE_TELEOP, false);
        advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
        CHECK(sendFrame(ctl, 0, 0, mode, true, false, false, INDICATOR_OFF,
                        false, SRC_TELEOP));
        CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    }
}

static void a_timeout_latches_even_if_nobody_looked_in_between() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS + 50);
    sendFrame(ctl, 500, 0, MODE_TELEOP, false);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
}

static void the_latched_reason_outlives_the_condition() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    sendFrame(ctl, 500, 0, MODE_TELEOP, false, false, false, INDICATOR_OFF,
              true);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK_EQ(ctl.latchedFaults(), FAULT_C2_LINK_LOST);

    sendFrame(ctl, 500, 0, MODE_TELEOP, false);
    CHECK_EQ(ctl.c2LinkState(), LINK_OK);
    CHECK(ctl.faultWord() & FAULT_C2_LINK_LOST);

    sendFrame(ctl, 0, 0, MODE_SAFE, false);
    CHECK(!(ctl.faultWord() & FAULT_C2_LINK_LOST));
}

static void the_state_frame_reports_fault() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false);
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    CHECK_EQ(ctl.buildState().mode, MODE_FAULT);
}

static void c2_loss_faults_teleop_but_not_autonomy() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false, false, false, INDICATOR_TELEOP);
    sendControl(ctl, 500, 0, MODE_TELEOP, false, false, false,
                INDICATOR_TELEOP, true);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK(ctl.effectiveStop());

    RoverController aut = freshController();
    sendControl(aut, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, true);
    CHECK_EQ(aut.activeMode(), MODE_AUTONOMOUS);
    CHECK(!aut.effectiveStop());
}

static void a_pause_or_abort_is_not_a_fault() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, true);
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, true);
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
}

static void fault_cannot_be_commanded() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    CHECK(!sendControl(ctl, 500, 0, MODE_FAULT, false));
    CHECK(ctl.faultWord() & FAULT_PROTOCOL_ERROR);
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
}

static void stop_decision_is_unchanged_by_active_mode() {
    const uint8_t modes[] = {MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS};
    const uint32_t ages[] = {0, DEFAULT_WATCHDOG_TIMEOUT_MS - 1,
                             DEFAULT_WATCHDOG_TIMEOUT_MS};
    for (int booted = 0; booted <= 1; ++booted)
    for (uint8_t mode : modes)
    for (int stop = 0; stop <= 1; ++stop)
    for (int abort = 0; abort <= 1; ++abort)
    for (int c2 = 0; c2 <= 1; ++c2)
    for (uint32_t age : ages) {
        RoverController ctl = freshController(0xFFFFFF80u);
        if (booted) {
            sendControl(ctl, 500, 0, mode, stop != 0, abort != 0, false,
                        INDICATOR_OFF, c2 != 0);
        }
        advance(age);
        const ControlMsg& m = ctl.lastControl();
        const bool old_formula = ctl.watchdogTripped()
            || m.stop
            || !modePermitsMotion(m.mode)
            || (m.autonomy_abort && m.mode == MODE_AUTONOMOUS)
            || (m.c2_lost && m.mode == MODE_TELEOP);
        CHECK_EQ(ctl.effectiveStop(), old_formula);
    }
}

static void the_indicator_follows_the_mode_the_controller_is_in() {
    RoverController ctl = freshController();

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.buildState().indicator_state, INDICATOR_AUTONOMOUS);

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_ARRIVED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_ARRIVED);

    sendControl(ctl, 500, 0, MODE_TELEOP, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_TELEOP);
    sendControl(ctl, 500, 0, MODE_TELEOP, false, false, false,
                INDICATOR_ARRIVED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_TELEOP);

    sendControl(ctl, 0, 0, MODE_SAFE, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_OFF);
}

static void the_indicator_shows_fault_when_nothing_is_driving() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS);
    advance(400);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_FAULT);
    CHECK_EQ(ctl.buildState().indicator_state, INDICATOR_FAULT);

    RoverController ctl2 = freshController();
    sendControl(ctl2, 500, 0, MODE_TELEOP, false, false, false, INDICATOR_TELEOP);
    sendControl(ctl2, 500, 0, MODE_TELEOP, false, false, false,
                INDICATOR_TELEOP, true);
    CHECK_EQ(ctl2.indicatorState(), INDICATOR_FAULT);
}

static void a_non_stopping_fault_does_not_blank_the_indicator() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS);
    TelemetryPower pw = ctl.buildPower(0, 2400, FAULT_UNDERVOLTAGE);
    CHECK((pw.fault_status & FAULT_UNDERVOLTAGE) != 0);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);
}

static void power_reports_the_same_fault_word_the_controller_acts_on() {
    RoverController ctl = freshController();
    TelemetryPower pw = ctl.buildPower(150, 2400, FAULT_OVER_CURRENT,
                                       FAULT_FIRMWARE_FAULT);
    CHECK(pw.fault_status & FAULT_OVER_CURRENT);
    CHECK(pw.fault_status & FAULT_FIRMWARE_FAULT);
    CHECK(pw.fault_status & FAULT_COMM_TIMEOUT);
    CHECK_EQ(pw.fault_status, ctl.faultWord(FAULT_OVER_CURRENT, FAULT_FIRMWARE_FAULT));
    CHECK_EQ(pw.current_ca, 150);
    CHECK_EQ(pw.voltage_cv, 2400);
}

static void drive_r_carries_the_command_age() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    advance(120);
    CHECK_EQ(ctl.buildDriveR(4242).cmd_age_ms, 120);
    CHECK_EQ(ctl.buildDriveR(4242).enc_right, 4242);
}

static void age_is_unknown_before_the_first_command() {
    RoverController ctl = freshController();
    CHECK_EQ(ctl.buildDriveR(0).cmd_age_ms, CMD_AGE_UNKNOWN);
    CHECK(ctl.buildPower(0, 0).fault_status & FAULT_COMM_TIMEOUT);
}

static void telemetry_paces_at_the_configured_period() {
    RoverController ctl = freshController();
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());
    advance(50);
    CHECK(ctl.telemetryDue());
}

static void the_telemetry_sequence_advances_once_per_cycle() {
    RoverController ctl = freshController();
    CHECK(ctl.telemetryDue());
    const uint8_t first = ctl.telemetrySeq();
    CHECK_EQ(ctl.telemetrySeq(), first);
    CHECK_EQ(ctl.telemetrySeq(), first);
    advance(50);
    CHECK(ctl.telemetryDue());
    CHECK_EQ(seqDelta(first, ctl.telemetrySeq()), 1);
}

static void telemetry_does_not_burst_after_a_stall() {
    RoverController ctl = freshController();
    ctl.telemetryDue();
    advance(5000);
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());
}

static void telemetry_pacing_is_correct_across_wraparound() {
    RoverController ctl = freshController(0xFFFFFFF0u);
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());
    advance(50);
    CHECK(g_now < 0xFFFFFFF0u);
    CHECK(ctl.telemetryDue());
}

static void an_autonomy_command_is_refused_while_teleoperated() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, 600, 200, MODE_TELEOP, false, false, false,
                       INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.lastDecodeResult(), DECODE_OK);
    CHECK_EQ(ctl.lastVerdict(), VERDICT_REJECT_WRONG_SOURCE);
    CHECK_EQ(ctl.controlFramesAccepted(), 2u);
    CHECK_EQ(ctl.framesRefused(), 1u);
    CHECK(!ctl.commandAuthorized());
    CHECK(!ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_WRONG_SOURCE);
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
}

static void a_teleop_command_is_refused_while_autonomous() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, 600, 200, MODE_AUTONOMOUS, false, false, false,
                       INDICATOR_OFF, false, SRC_TELEOP));
    CHECK_EQ(ctl.lastVerdict(), VERDICT_REJECT_WRONG_SOURCE);
    CHECK(!ctl.commandAuthorized());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_WRONG_SOURCE);
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
}

static void the_same_command_is_accepted_once_the_mode_matches() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, 600, 200, MODE_TELEOP, false, false, false,
                       INDICATOR_OFF, false, SRC_AUTONOMY));
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);

    CHECK(sendControl(ctl, 600, 200, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK(ctl.commandAuthorized());
    ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 600); CHECK_EQ(s, 200);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_APPLIED);
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
    advance(REJECT_REPORT_HOLD_MS);
    sendControl(ctl, 600, 200, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_OFF, false, SRC_AUTONOMY);
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));
}

static void stop_overrides_an_authorised_autonomy_command() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 900, 0, MODE_AUTONOMOUS, true, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK(ctl.commandAuthorized());
    CHECK(ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
}

static void a_stop_from_the_wrong_source_still_stops_the_rover() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 900, 0, MODE_AUTONOMOUS, true, false, false,
                      INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(!ctl.commandAuthorized());
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
}

static void an_abort_from_the_wrong_source_still_stops_the_rover() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 900, 0, MODE_AUTONOMOUS, false, true,
                      false, INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

static void disabled_mode_has_no_owner_so_neither_source_mismatches() {
    for (int src = 0; src <= 1; ++src) {
        RoverController ctl = freshController();
        CHECK(sendControl(ctl, 0, 0, MODE_SAFE, false, false, false,
                          INDICATOR_OFF, false,
                          src ? SRC_AUTONOMY : SRC_TELEOP));
        CHECK(ctl.commandAuthorized());
        CHECK(!ctl.sourceMismatch());
        CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));
        CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
    }
}

// The wrong-sender fault stays on after the timeout it caused, so the operator
// can see why the rover stopped. It clears after REJECT_REPORT_HOLD_MS.
static void a_source_mismatch_is_held_then_expires() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                       INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(ctl.sourceMismatch());
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);

    advance(400);
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.sourceMismatch());
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);

    advance(REJECT_REPORT_HOLD_MS);
    CHECK(!ctl.sourceMismatch());
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));
}

// A3: a command from the wrong sender must not reset the timeout.
// (Before Task 6 it did.)
static void an_unauthorised_frame_does_not_feed_the_watchdog() {
    RoverController ctl = freshController();
    grantMode(ctl, MODE_AUTONOMOUS);
    advance(200);
    CHECK(!sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                     INDICATOR_OFF, false, SRC_TELEOP));
    advance(100);
    CHECK(!sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                     INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
}

static void an_arrival_claim_from_the_wrong_source_is_ignored() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                       INDICATOR_ARRIVED, false, SRC_TELEOP));
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);

    CHECK(sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_ARRIVED, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.indicatorState(), INDICATOR_ARRIVED);
}

static void a_return_request_is_reported_but_changes_no_behaviour() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, true,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK(ctl.returnRequestActive());
    CHECK(ctl.returnRequest());
    CHECK(!ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 500);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);
}

static void a_return_request_outside_autonomous_is_not_active() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 0, MODE_TELEOP, false, false, true));
    CHECK(ctl.returnRequest());
    CHECK(!ctl.returnRequestActive());
}

static void a_return_request_from_the_operator_is_honoured() {
    RoverController ctl = freshController();
    // The command is rejected, but the return request still counts:
    // URC rule 1.e.xi says the operator is the one who signals the return.
    CHECK(!sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, true,
                       INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(!ctl.commandAuthorized());
    CHECK(ctl.returnRequestActive());
}

static void a_stale_return_request_is_not_reported() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, true,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK(ctl.returnRequestActive());
    advance(400);
    CHECK(!ctl.returnRequestActive());
}

static void the_echo_acknowledges_the_sequence_number_it_acted_on() {
    RoverController ctl = freshController();
    CHECK(sendWithSeq(ctl, 76, 0));
    CHECK(sendWithSeq(ctl, 77, 400));
    TelemetryCmdEcho e = ctl.buildCmdEcho(400, 0);
    CHECK_EQ(e.ack_seq, 77);
    CHECK_EQ(e.drive_applied, 400);
    CHECK_EQ(cmdStatusOf(e.status), CMD_STATUS_APPLIED);
    CHECK_EQ(decodeResultOf(e.status), DECODE_OK);
    CHECK(!returnActiveOf(e.status));
}

static void the_echo_reports_no_command_before_the_first_frame() {
    RoverController ctl = freshController();
    TelemetryCmdEcho e = ctl.buildCmdEcho(0, 0);
    CHECK_EQ(cmdStatusOf(e.status), CMD_STATUS_NO_COMMAND);
}

static void the_echo_carries_the_reason_the_last_frame_was_rejected() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 400, 0, MODE_TELEOP, false));
    uint8_t buf[8] = {0};
    ControlMsg m;
    m.drive_cmd = 400; m.steer_cmd = 0; m.mode = MODE_TELEOP;
    m.stop = false; m.autonomy_abort = false; m.return_request = false;
    m.c2_lost = false; m.from_autonomy = false;
    m.indicator_request = INDICATOR_OFF; m.seq = ++g_seq;
    encodeControl(m, buf);
    buf[0] ^= 0xFF;
    CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, FRAME_DLC));

    TelemetryCmdEcho e = ctl.buildCmdEcho(400, 0);
    CHECK_EQ(decodeResultOf(e.status), DECODE_BAD_CRC);
    CHECK_EQ(cmdStatusOf(e.status), CMD_STATUS_APPLIED);
    CHECK_EQ(e.drive_applied, 400);
}

static void the_echo_reports_a_refused_command_as_zero() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, 800, 300, MODE_TELEOP, false, false, false,
                       INDICATOR_OFF, false, SRC_AUTONOMY));
    int16_t d, s; ctl.commandedOutputs(d, s);
    TelemetryCmdEcho e = ctl.buildCmdEcho(d, s);
    CHECK_EQ(e.drive_applied, 0);
    CHECK_EQ(e.steer_applied, 0);
    CHECK_EQ(cmdStatusOf(e.status), CMD_STATUS_WRONG_SOURCE);
}

static void the_echo_reports_an_active_return_request() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 300, 0, MODE_AUTONOMOUS, false, false, true,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    TelemetryCmdEcho e = ctl.buildCmdEcho(300, 0);
    CHECK(returnActiveOf(e.status));
}

static void outputs_are_nonzero_only_when_permitted_and_authorised() {
    int moved = 0, refused = 0, stopped_count = 0;
    for (int mode = 0; mode <= 3; ++mode) {
        for (int src = 0; src <= 1; ++src) {
            for (int stop = 0; stop <= 1; ++stop) {
                for (int abort = 0; abort <= 1; ++abort) {
                    for (int c2 = 0; c2 <= 1; ++c2) {
                        for (int stale = 0; stale <= 1; ++stale) {
                            RoverController ctl = freshController();
                            const bool honoured = sendControl(
                                ctl, 700, 300, static_cast<uint8_t>(mode),
                                stop != 0, abort != 0, false, INDICATOR_OFF,
                                c2 != 0, src ? SRC_AUTONOMY : SRC_TELEOP);
                            const uint8_t verdict = ctl.lastVerdict();
                            CHECK_EQ(honoured, verdict == VERDICT_ACCEPTED
                                               || verdict == VERDICT_STOP);
                            if (mode > MODE_MAX) {
                                CHECK_EQ(verdict, VERDICT_REJECT_MALFORMED);
                            }
                            if (stale) advance(400);

                            int16_t d, s;
                            ctl.commandedOutputs(d, s);
                            const bool moving = d != 0 || s != 0;
                            CHECK_EQ(moving,
                                     ctl.commandStatus() == CMD_STATUS_APPLIED);

                            if (moving) {
                                CHECK(!stale);
                                CHECK(!stop);
                                CHECK(!ctl.effectiveStop());
                                CHECK_EQ(verdict, VERDICT_ACCEPTED);
                                CHECK(modePermitsMotion(ctl.activeMode()));
                                CHECK(sourceOwnsMode(src != 0, ctl.activeMode()));
                                CHECK_EQ(d, 700); CHECK_EQ(s, 300);
                                moved++;
                            } else if (verdictIsRejection(verdict)) {
                                refused++;
                            } else {
                                stopped_count++;
                            }

                            CHECK_EQ((ctl.faultWord() & FAULT_CMD_SOURCE) != 0,
                                     ctl.sourceMismatch());
                            CHECK_EQ((ctl.faultWord() & FAULT_MODE_CONFLICT) != 0,
                                     ctl.modeConflict());
                            if (stale) CHECK(ctl.watchdogTripped());
                        }
                    }
                }
            }
        }
    }
    CHECK(moved > 0);
    CHECK(refused > 0);
    CHECK(stopped_count > 0);
}

static void a_refused_command_is_not_a_stop() {
    RoverController ctl = freshController();
    CHECK(!sendControl(ctl, 700, 300, MODE_AUTONOMOUS, false, false, false,
                       INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(!ctl.effectiveStop());
    CHECK(!ctl.stopLatched());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_WRONG_SOURCE);

    CHECK(sendControl(ctl, 700, 300, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 700); CHECK_EQ(s, 300);
}

static void autonomy_cannot_put_itself_in_charge() {
    RoverController ctl = freshController();
    CHECK(!sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                     INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK(ctl.modeRefused());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);
    CHECK_EQ(ctl.buildState().mode, MODE_SAFE);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));
    CHECK(ctl.faultWord() & FAULT_MODE_CONFLICT);
    CHECK(ctl.watchdogTripped());
}

static void autonomy_cannot_take_over_from_the_operator() {
    RoverController ctl = freshController();
    sendFrame(ctl, 300, 0, MODE_TELEOP, false);
    CHECK(!sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                     INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);
}

static void a_refused_mode_change_does_not_drive_in_the_old_mode() {
    RoverController ctl = freshController();
    grantMode(ctl, MODE_AUTONOMOUS);
    CHECK(!sendFrame(ctl, 600, 0, MODE_TELEOP, false, false, false,
                     INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
    CHECK(!ctl.commandAuthorized());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK(!ctl.effectiveStop());
}

// CHANGED: the operator hands over at neutral, without STOP.
static void the_operator_hands_over_and_autonomy_drives() {
    RoverController ctl = freshController();
    CHECK(sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                    INDICATOR_OFF, false, SRC_TELEOP));
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
    CHECK(!ctl.stopLatched());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_APPLIED);
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));

    CHECK(sendFrame(ctl, 600, 100, MODE_AUTONOMOUS, false));
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_APPLIED);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 600); CHECK_EQ(s, 100);
}

static void the_operator_can_switch_between_the_motion_modes() {
    RoverController ctl = freshController();
    sendFrame(ctl, 0, 0, MODE_TELEOP, false);
    grantMode(ctl, MODE_AUTONOMOUS);
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
    sendFrame(ctl, 0, 0, MODE_TELEOP, false);
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
}

static void either_source_can_stand_the_rover_down() {
    const uint8_t modes[] = {MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        for (int src = 0; src <= 1; ++src) {
            RoverController ctl = freshController();
            sendControl(ctl, 500, 0, mode, false);
            CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false, false, false,
                            INDICATOR_OFF, false,
                            src ? SRC_AUTONOMY : SRC_TELEOP));
            CHECK_EQ(ctl.activeMode(), MODE_SAFE);
        }
    }
}

static void teleop_is_not_entered_while_c2_is_lost() {
    RoverController ctl = freshController();
    sendFrame(ctl, 500, 0, MODE_TELEOP, false, false, false, INDICATOR_OFF,
              true);
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);
}

static void a_refusal_is_not_reported_once_it_is_stale() {
    RoverController ctl = freshController();
    sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false, INDICATOR_OFF,
              false, SRC_AUTONOMY);
    CHECK(ctl.modeRefused());
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    CHECK(!ctl.modeRefused());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
}

static void a_critical_fault_faults_a_moving_rover() {
    const uint16_t critical[] = {FAULT_OVER_CURRENT, FAULT_ESTOP_ACTIVE};
    for (uint16_t bit : critical) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, MODE_TELEOP, false);
        ctl.updateFaults(bit, CTRL_HEALTH_OK);
        CHECK_EQ(ctl.activeMode(), MODE_FAULT);
        CHECK(ctl.effectiveStop());
        CHECK_EQ(ctl.latchedFaults(), bit);

        ctl.updateFaults(0, CTRL_HEALTH_OK);
        CHECK_EQ(ctl.activeMode(), MODE_FAULT);
        CHECK(ctl.faultWord() & bit);
    }
}

static void a_critical_fault_in_safe_does_not_latch_but_blocks_driving() {
    RoverController ctl = freshController();
    sendFrame(ctl, 0, 0, MODE_SAFE, false);
    ctl.updateFaults(FAULT_OVER_CURRENT, CTRL_HEALTH_OK);
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);

    sendFrame(ctl, 500, 0, MODE_TELEOP, false);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0);
    CHECK(ctl.faultWord() & FAULT_OVER_CURRENT);
}

static void a_drive_stage_fault_faults_a_moving_rover() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false);
    ctl.updateFaults(0, CTRL_HEALTH_FAULT);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
}

static void safe_is_refused_while_the_fault_is_still_there() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    ctl.updateFaults(FAULT_OVER_CURRENT, CTRL_HEALTH_OK);

    CHECK(!sendFrame(ctl, 0, 0, MODE_SAFE, false));
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);

    ctl.updateFaults(0, CTRL_HEALTH_OK);
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false));
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
}

static void non_critical_faults_are_reported_but_do_not_stop_the_rover() {
    const uint16_t minor[] = {FAULT_UNDERVOLTAGE, FAULT_ENCODER_FAULT,
                              FAULT_FIRMWARE_FAULT};
    for (uint16_t bit : minor) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, MODE_TELEOP, false);
        ctl.updateFaults(bit, CTRL_HEALTH_DEGRADED);
        CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
        int16_t d, s; ctl.commandedOutputs(d, s);
        CHECK_EQ(d, 500);
        CHECK(ctl.buildPower(0, 2400, bit).fault_status & bit);
    }
}

static void commanded_is_what_the_drive_stage_is_handed() {
    RoverController ctl = freshController();
    CHECK_EQ(ctl.getCommandedDrive(), 0);
    CHECK_EQ(ctl.getCommandedSteer(), 0);

    sendControl(ctl, 500, 100, MODE_TELEOP, false);
    CHECK_EQ(ctl.getCommandedDrive(), 500);
    CHECK_EQ(ctl.getCommandedSteer(), 100);

    sendControl(ctl, 500, 100, MODE_TELEOP, true);
    CHECK_EQ(ctl.getCommandedDrive(), 0);
    CHECK_EQ(ctl.getCommandedSteer(), 0);

    sendFrame(ctl, 500, 100, MODE_TELEOP, false, false, false, INDICATOR_OFF,
              false, SRC_AUTONOMY);
    CHECK_EQ(ctl.getCommandedDrive(), 0);

    sendFrame(ctl, 500, 100, MODE_AUTONOMOUS, false, false, false,
              INDICATOR_OFF, false, SRC_AUTONOMY);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);
    CHECK_EQ(ctl.getCommandedDrive(), 0);
    CHECK_EQ(ctl.getCommandedSteer(), 0);
}

static void measured_is_whatever_the_plant_last_reported() {
    RoverController ctl = freshController();
    CHECK_EQ(ctl.getMeasuredDrive(), 0);
    CHECK_EQ(ctl.getMeasuredSteer(), 0);

    ctl.updateMeasured(321, -45);
    CHECK_EQ(ctl.getMeasuredDrive(), 321);
    CHECK_EQ(ctl.getMeasuredSteer(), -45);

    sendControl(ctl, 500, 100, MODE_TELEOP, false);
    CHECK_EQ(ctl.getMeasuredDrive(), 321);
    CHECK_EQ(ctl.getCommandedDrive(), 500);
    ctl.updateMeasured(999, 999);
    CHECK_EQ(ctl.getCommandedDrive(), 500);
    CHECK_EQ(ctl.getCommandedSteer(), 100);
}

struct Plant {
    DriveModule drv;
    DriveSim    sim;
    int16_t     steer_fb = 0;
};

static void runLoop(RoverController& ctl, Plant& p, uint32_t ms, int16_t drive,
                    int16_t steer, bool stop, bool stalled = false) {
    for (uint32_t i = 0; i < ms; ++i) {
        if (i % 50 == 0) sendFrame(ctl, drive, steer, MODE_TELEOP, stop);
        advance(1);
        int16_t d, s;
        ctl.commandedOutputs(d, s);
        const DriveOutputs& out = p.drv.update(d, s, !ctl.effectiveStop());
        p.sim.update(out.left_effort, out.right_effort,
                     out.energized && !stalled, g_now);
        if (!stalled) {
            if (p.steer_fb < out.steer_effort)
                p.steer_fb = static_cast<int16_t>(p.steer_fb + 25 < out.steer_effort
                                                  ? p.steer_fb + 25 : out.steer_effort);
            else if (p.steer_fb > out.steer_effort)
                p.steer_fb = static_cast<int16_t>(p.steer_fb - 25 > out.steer_effort
                                                  ? p.steer_fb - 25 : out.steer_effort);
        }
        ctl.updateMeasured(
            scaledFromMps((p.sim.left().measured_mps + p.sim.right().measured_mps) / 2.0f),
            p.steer_fb);
    }
}

static int gap(int16_t a, int16_t b) { return a > b ? a - b : b - a; }

static void measured_follows_commanded_in_the_closed_loop() {
    RoverController ctl = freshController();
    sendControl(ctl, 0, 0, MODE_TELEOP, false);
    Plant p;

    runLoop(ctl, p, 10, 500, 200, false);
    CHECK_EQ(ctl.getCommandedDrive(), 500);
    CHECK(gap(ctl.getCommandedDrive(), ctl.getMeasuredDrive()) > 300);

    runLoop(ctl, p, 1000, 500, 200, false);
    CHECK(gap(ctl.getCommandedDrive(), ctl.getMeasuredDrive()) <= 10);
    CHECK_EQ(ctl.getMeasuredSteer(), 200);

    runLoop(ctl, p, 1, 500, 200, true);
    CHECK_EQ(ctl.getCommandedDrive(), 0);
    CHECK(ctl.getMeasuredDrive() > 0);
    runLoop(ctl, p, 500, 500, 200, true);
    CHECK_EQ(ctl.getMeasuredDrive(), 0);
}

static void a_stalled_drivetrain_leaves_the_gap_open() {
    RoverController ctl = freshController();
    sendControl(ctl, 0, 0, MODE_TELEOP, false);
    Plant p;
    runLoop(ctl, p, 1000, 500, 200, false, true);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_APPLIED);
    CHECK_EQ(ctl.getCommandedDrive(), 500);
    CHECK_EQ(ctl.getMeasuredDrive(), 0);
    CHECK_EQ(ctl.getMeasuredSteer(), 0);
    CHECK(gap(ctl.getCommandedSteer(), ctl.getMeasuredSteer()) == 200);
}

static void freshFeedback(RoverController& ctl) {
    ctl.updateFeedback(g_now, true, g_now, true);
}

static void a_channel_that_never_reported_is_unknown_not_zero() {
    RoverController ctl = freshController();
    CHECK_EQ(ctl.encoderAgeMs(), FEEDBACK_AGE_UNKNOWN);
    CHECK_EQ(ctl.steerAgeMs(), FEEDBACK_AGE_UNKNOWN);
    CHECK(ctl.encoderStale());
    CHECK(ctl.steerStale());
    CHECK(!ctl.encoderInvalid());
    CHECK(!ctl.steerInvalid());
}

static void the_age_is_measured_from_the_last_update() {
    RoverController ctl = freshController(1000);
    freshFeedback(ctl);
    CHECK_EQ(ctl.encoderAgeMs(), static_cast<uint16_t>(0));
    advance(40);
    CHECK_EQ(ctl.encoderAgeMs(), static_cast<uint16_t>(40));
    CHECK_EQ(ctl.steerAgeMs(), static_cast<uint16_t>(40));
    CHECK(!ctl.encoderStale());
    CHECK(!ctl.steerStale());
}

static void a_frozen_channel_goes_stale_while_the_other_stays_fresh() {
    RoverController ctl = freshController(1000);
    const uint32_t frozen_at = g_now;
    freshFeedback(ctl);
    for (uint32_t i = 0; i < 20; ++i) {
        advance(10);
        ctl.updateFeedback(frozen_at, true, g_now, true);
    }
    CHECK(ctl.encoderStale());
    CHECK(!ctl.steerStale());
    CHECK_EQ(ctl.encoderAgeMs(), static_cast<uint16_t>(200));
    CHECK_EQ(ctl.steerAgeMs(), static_cast<uint16_t>(0));
}

static void staleness_trips_exactly_at_the_boundary() {
    RoverController ctl = freshController(1000);
    const uint32_t at = g_now;
    freshFeedback(ctl);
    advance(FEEDBACK_STALE_MS - 1);
    CHECK(!ctl.encoderStale());
    advance(1);
    CHECK(ctl.encoderStale());
    ctl.updateFeedback(g_now, true, g_now, true);
    CHECK(!ctl.encoderStale());
    (void)at;
}

static void the_age_is_correct_across_millis_wraparound() {
    RoverController ctl = freshController(0xFFFFFFF0u);
    freshFeedback(ctl);
    advance(40);
    CHECK_EQ(ctl.encoderAgeMs(), static_cast<uint16_t>(40));
    CHECK(!ctl.encoderStale());
}

static void a_very_old_reading_saturates_without_colliding_with_unknown() {
    RoverController ctl = freshController(1000);
    const uint32_t at = g_now;
    freshFeedback(ctl);
    advance(300000);
    ctl.updateFeedback(at, true, at, true);
    CHECK_EQ(ctl.encoderAgeMs(), FEEDBACK_AGE_MAX);
    CHECK(ctl.encoderAgeMs() != FEEDBACK_AGE_UNKNOWN);
}

static void an_impossible_reading_is_invalid_but_not_stale() {
    RoverController ctl = freshController(1000);
    ctl.updateFeedback(g_now, false, g_now, true);
    CHECK(ctl.encoderInvalid());
    CHECK(!ctl.encoderStale());
    CHECK(!ctl.steerInvalid());
}

static void stale_and_invalid_reach_the_fault_word() {
    RoverController ctl = freshController(1000);
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    freshFeedback(ctl);
    CHECK((ctl.faultWord() & FAULT_STALE_FEEDBACK) == 0);
    CHECK((ctl.faultWord() & FAULT_ENCODER_FAULT) == 0);
    CHECK((ctl.faultWord() & FAULT_STEERING_FAULT) == 0);

    const uint32_t at = g_now;
    advance(FEEDBACK_STALE_MS);
    ctl.updateFeedback(at, false, at, false);
    CHECK((ctl.faultWord() & FAULT_STALE_FEEDBACK) != 0);
    CHECK((ctl.faultWord() & FAULT_ENCODER_FAULT) != 0);
    CHECK((ctl.faultWord() & FAULT_STEERING_FAULT) != 0);
}

static void stale_feedback_does_not_stop_the_rover() {
    RoverController ctl = freshController(1000);
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    freshFeedback(ctl);
    CHECK(!ctl.effectiveStop());
    const uint32_t at = g_now;
    advance(FEEDBACK_STALE_MS + 50);
    ctl.updateFeedback(at, false, at, false);
    CHECK(!ctl.effectiveStop());
    int16_t drive, steer;
    ctl.commandedOutputs(drive, steer);
    CHECK_EQ(drive, static_cast<int16_t>(400));
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
}

static void the_source_is_none_before_anybody_has_spoken() {
    RoverController ctl = freshController();
    CHECK_EQ(ctl.commandSource(), CMD_SOURCE_NONE);
}

static void the_source_names_the_producer_that_is_driving() {
    RoverController ctl = freshController(1000);
    sendControl(ctl, 400, 0, MODE_TELEOP, false, false, false, INDICATOR_OFF,
                false, SRC_TELEOP);
    CHECK_EQ(ctl.commandSource(), CMD_SOURCE_OPERATOR);
    sendControl(ctl, 400, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_OFF, false, SRC_AUTONOMY);
    CHECK_EQ(ctl.commandSource(), CMD_SOURCE_AUTONOMY);
}

// A rejected command changes nothing: the operator keeps driving.
static void a_refused_source_does_not_displace_the_active_source() {
    RoverController ctl = freshController(1000);
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    sendControl(ctl, 900, 0, MODE_TELEOP, false, false, false, INDICATOR_OFF,
                false, SRC_AUTONOMY);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_WRONG_SOURCE);
    CHECK_EQ(ctl.commandSource(), CMD_SOURCE_OPERATOR);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 400);
}

static void safe_has_no_owner_so_it_has_no_source() {
    RoverController ctl = freshController(1000);
    sendControl(ctl, 0, 0, MODE_SAFE, false, false, false, INDICATOR_OFF,
                false, SRC_TELEOP);
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK_EQ(ctl.commandSource(), CMD_SOURCE_NONE);
}

static void a_stale_source_is_not_reported() {
    RoverController ctl = freshController(1000);
    sendControl(ctl, 400, 0, MODE_TELEOP, false, false, false, INDICATOR_OFF,
                false, SRC_TELEOP);
    CHECK_EQ(ctl.commandSource(), CMD_SOURCE_OPERATOR);
    advance(301);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.commandSource(), CMD_SOURCE_NONE);
}

static void the_feedback_frame_carries_what_the_accessors_say() {
    RoverController ctl = freshController(1000);
    sendControl(ctl, 400, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_OFF, false, SRC_AUTONOMY);
    const uint32_t at = g_now;
    ctl.updateFeedback(at, true, at, false);
    advance(FEEDBACK_STALE_MS + 10);
    ctl.updateFeedback(at, true, g_now, false);

    const TelemetryFeedback m = ctl.buildFeedback();
    CHECK_EQ(m.encoder_age_ms, ctl.encoderAgeMs());
    CHECK_EQ(m.steer_age_ms, ctl.steerAgeMs());
    CHECK_EQ(m.cmd_source, ctl.commandSource());
    CHECK((m.flags & FB_FLAG_ENCODER_STALE) != 0);
    CHECK((m.flags & FB_FLAG_STEER_STALE) == 0);
    CHECK((m.flags & FB_FLAG_STEER_INVALID) != 0);
    CHECK((m.flags & FB_FLAGS_RESERVED_MASK) == 0);
}

static void the_feedback_frame_round_trips_on_the_wire() {
    RoverController ctl = freshController(1000);
    sendControl(ctl, 400, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_OFF, false, SRC_AUTONOMY);
    ctl.updateFeedback(g_now, true, g_now, true);
    const TelemetryFeedback sent = ctl.buildFeedback();

    uint8_t buf[8] = {0};
    const uint8_t n = encodeTelemetryFeedback(sent, 77, buf);
    CHECK_EQ(n, static_cast<uint8_t>(FRAME_DLC));

    TelemetryFeedback got; uint8_t seq = 0;
    CHECK_EQ(decodeTelemetryFeedback(CAN_ID_TELEM_FEEDBACK, buf, n, got, seq),
             DECODE_OK);
    CHECK_EQ(seq, static_cast<uint8_t>(77));
    CHECK_EQ(got.encoder_age_ms, sent.encoder_age_ms);
    CHECK_EQ(got.steer_age_ms, sent.steer_age_ms);
    CHECK_EQ(got.cmd_source, sent.cmd_source);
    CHECK_EQ(got.flags, sent.flags);
}

// ===========================================================================
// Task 6 Group A: who is allowed to command the rover.
// Gate 1 intact -> gate 2 well formed -> gate 3 allowed -> gate 4 safe to move.
// STOP only needs gates 1 and 2.
// ===========================================================================

static bool sendCorrupt(RoverController& ctl, int16_t drive, uint8_t mode,
                        bool stop, Source src = SRC_OWNER) {
    ControlMsg m;
    m.drive_cmd = drive; m.steer_cmd = 0; m.mode = mode;
    m.stop = stop; m.autonomy_abort = false; m.return_request = false;
    m.c2_lost = false; m.from_autonomy = sourceBit(src, mode);
    m.indicator_request = INDICATOR_OFF; m.seq = ++g_seq;
    uint8_t buf[8] = {0};
    encodeControl(m, buf);
    buf[FRAME_CRC_OFFSET] ^= 0x5A;
    return ctl.ingestFrame(CAN_ID_CONTROL, buf, FRAME_DLC);
}

static int16_t outputDrive(const RoverController& ctl) {
    int16_t d, s;
    ctl.commandedOutputs(d, s);
    return d;
}

// A1: with one arbiter, a second sender can't make the motors flicker.
// (Before Task 6 this went 400, 0, 400, 0 ...)
static void the_arbiter_does_not_flap_between_two_producers() {
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    for (int i = 0; i < 6; ++i) {
        advance(25);
        CHECK(!sendFrame(ctl, 900, 0, MODE_TELEOP, false, false, false,
                         INDICATOR_OFF, false, SRC_AUTONOMY));
        CHECK_EQ(outputDrive(ctl), 400);
        advance(25);
        CHECK(sendFrame(ctl, 400, 0, MODE_TELEOP, false));
        CHECK_EQ(outputDrive(ctl), 400);
    }
    CHECK(!ctl.watchdogTripped());
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
}

// A2: a command can be readable and still be wrong.
static void motion_in_safe_is_a_mode_conflict() {
    RoverController ctl = freshController();
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false));
    CHECK(!sendFrame(ctl, 600, 0, MODE_SAFE, false));
    CHECK_EQ(ctl.lastVerdict(), VERDICT_REJECT_MODE_CONFLICT);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_CONFLICT);
    CHECK(ctl.faultWord() & FAULT_MODE_CONFLICT);
    CHECK_EQ(outputDrive(ctl), 0);
}

static void an_invalid_command_does_not_replace_the_active_command() {
    RoverController ctl = freshController();
    sendControl(ctl, 400, 100, MODE_TELEOP, false);
    CHECK(!sendFrame(ctl, CMD_MAX + 1, 0, MODE_TELEOP, false));
    CHECK_EQ(ctl.lastVerdict(), VERDICT_REJECT_MALFORMED);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 400); CHECK_EQ(s, 100);
    CHECK(ctl.faultWord() & FAULT_PROTOCOL_ERROR);
}

// A3: only an accepted command (or a STOP) resets the timeout.
static void a_refused_mode_change_does_not_feed_the_watchdog() {
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    advance(200);
    CHECK(!sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                     INDICATOR_OFF, false, SRC_AUTONOMY));
    advance(100);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
}

static void a_motion_command_in_safe_does_not_feed_the_watchdog() {
    RoverController ctl = freshController();
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false));
    advance(200);
    CHECK(!sendFrame(ctl, 600, 0, MODE_SAFE, false));
    advance(100);
    CHECK(ctl.watchdogTripped());
}

static void a_stop_feeds_the_watchdog() {
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    for (int i = 0; i < 4; ++i) {
        advance(200);
        CHECK(sendFrame(ctl, 0, 0, MODE_TELEOP, true));
    }
    CHECK(!ctl.watchdogTripped());
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
    CHECK(ctl.effectiveStop());
}

static void a_jetson_sending_only_refused_frames_is_alive_but_stale() {
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    for (int i = 0; i < 8; ++i) {
        advance(50);
        sendFrame(ctl, 400, 0, MODE_TELEOP, false, false, false,
                  INDICATOR_OFF, false, SRC_AUTONOMY);
    }
    CHECK(ctl.watchdogTripped());
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_LOST);
}

static void entering_safe_clears_the_refusal_reports() {
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    sendFrame(ctl, 600, 0, MODE_TELEOP, false, false, false, INDICATOR_OFF,
              false, SRC_AUTONOMY);
    sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false, INDICATOR_OFF,
              false, SRC_AUTONOMY);
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
    CHECK(ctl.faultWord() & FAULT_MODE_CONFLICT);
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false));
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));
    CHECK(!(ctl.faultWord() & FAULT_MODE_CONFLICT));
}

// A4: STOP wins right away, and autonomy's next drive command can't undo it.
static void stop_is_not_overridden_by_the_next_autonomy_command() {
    RoverController ctl = freshController();
    sendControl(ctl, 600, 0, MODE_AUTONOMOUS, false);
    CHECK_EQ(outputDrive(ctl), 600);
    CHECK(sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, true, false, false,
                    INDICATOR_OFF, false, SRC_TELEOP));
    CHECK_EQ(outputDrive(ctl), 0);
    for (int i = 0; i < 5; ++i) {
        advance(50);
        CHECK(sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false));
        CHECK_EQ(outputDrive(ctl), 0);
        CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
    }
}

static void the_rover_boots_with_stop_latched() {
    RoverController ctl = freshController();
    CHECK(ctl.stopLatched());
    CHECK(sendFrame(ctl, 600, 0, MODE_TELEOP, false));
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
    CHECK_EQ(outputDrive(ctl), 0);
    CHECK(sendFrame(ctl, 0, 0, MODE_TELEOP, false));
    CHECK(sendFrame(ctl, 600, 0, MODE_TELEOP, false));
    CHECK_EQ(outputDrive(ctl), 600);
}

// CHANGED: after an abort, autonomy can't restart itself. The operator
// releases it, here by taking TELEOP at neutral.
static void an_abort_latches_like_a_stop_while_autonomous() {
    RoverController ctl = freshController();
    sendControl(ctl, 600, 0, MODE_AUTONOMOUS, false);
    CHECK(sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false, true));
    CHECK_EQ(ctl.lastVerdict(), VERDICT_STOP);
    CHECK(sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, false));
    CHECK(ctl.stopLatched());
    CHECK(sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false));
    CHECK_EQ(outputDrive(ctl), 0);

    CHECK(sendFrame(ctl, 0, 0, MODE_TELEOP, false));
    CHECK(!ctl.stopLatched());
    CHECK(sendFrame(ctl, 400, 0, MODE_TELEOP, false));
    CHECK_EQ(outputDrive(ctl), 400);
}

// NEW (Systems): only the operator can release STOP, no matter who sent it.
static void only_the_operator_can_release_stop() {
    // In SAFE, autonomy's neutral commands are accepted but don't release it.
    RoverController ctl = freshController();
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, true, false, false, INDICATOR_OFF,
                    false, SRC_AUTONOMY));
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false, false, false, INDICATOR_OFF,
                    false, SRC_AUTONOMY));
    CHECK(ctl.stopLatched());
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false, false, false, INDICATOR_OFF,
                    false, SRC_TELEOP));
    CHECK(!ctl.stopLatched());

    // In AUTONOMOUS, autonomy can't release even its own STOP. The operator
    // takes TELEOP (which releases it), then hands AUTONOMOUS back.
    sendControl(ctl, 600, 0, MODE_AUTONOMOUS, false);
    CHECK(sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, true));
    CHECK(sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, false));
    CHECK(ctl.stopLatched());
    grantMode(ctl, MODE_TELEOP);
    CHECK(!ctl.stopLatched());
    grantMode(ctl, MODE_AUTONOMOUS);
    CHECK(sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false));
    CHECK_EQ(outputDrive(ctl), 600);
}

// NEW: why the Jetson must hand over WITHOUT STOP now. A handover with STOP
// set latches the operator's STOP, and autonomy can't release it.
static void a_handover_with_stop_set_keeps_autonomy_stopped() {
    RoverController ctl = freshController();
    CHECK(sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, true, false, false,
                    INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(sendFrame(ctl, 0, 0, MODE_AUTONOMOUS, false));
    CHECK(sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false));
    CHECK(ctl.stopLatched());
    CHECK_EQ(outputDrive(ctl), 0);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
}

// STOP skips gate 3 but must pass gates 1 and 2. A damaged or invalid
// frame is ignored, even if its stop bit is set.
static void a_stop_must_pass_gates_one_and_two() {
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_TELEOP, false);

    CHECK(!sendCorrupt(ctl, 0, MODE_TELEOP, true));
    CHECK_EQ(ctl.lastVerdict(), VERDICT_REJECT_CORRUPT);
    CHECK(!ctl.stopLatched());
    CHECK_EQ(outputDrive(ctl), 400);

    CHECK(!sendFrame(ctl, CMD_MAX + 1, 0, MODE_TELEOP, true));
    CHECK_EQ(ctl.lastVerdict(), VERDICT_REJECT_MALFORMED);
    CHECK(!ctl.stopLatched());
    CHECK_EQ(outputDrive(ctl), 400);

    CHECK(sendFrame(ctl, 900, 0, MODE_TELEOP, true, false, false,
                    INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.lastVerdict(), VERDICT_STOP);
    CHECK_EQ(outputDrive(ctl), 0);
}

// A5: STOP stops the rover but does not change its mode.
static void stop_is_not_a_mode_change() {
    const uint8_t modes[] = {MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, mode, false);
        CHECK(sendFrame(ctl, 500, 0, mode, true));
        CHECK_EQ(ctl.activeMode(), mode);
        CHECK(ctl.effectiveStop());
    }
}

static void a_stop_frame_still_obeys_the_mode_rules() {
    RoverController ctl = freshController();
    sendControl(ctl, 600, 0, MODE_AUTONOMOUS, false);
    CHECK(sendFrame(ctl, 0, 0, MODE_TELEOP, true, false, false,
                    INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.lastVerdict(), VERDICT_STOP);
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
    CHECK(ctl.effectiveStop());
}

// A6 (Systems chose RIDE_OUT): a bad command arrives while the rover is moving.
static void an_invalid_command_while_moving_rides_out_to_the_timeout() {
    RoverController ctl = freshController();
    CHECK_EQ(ctl.invalidCommandPolicy(), INVALID_CMD_RIDE_OUT);
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    CHECK_EQ(outputDrive(ctl), 500);

    CHECK(!sendFrame(ctl, CMD_MAX + 1, 0, MODE_TELEOP, false));
    CHECK(ctl.faultWord() & FAULT_PROTOCOL_ERROR);
    CHECK_EQ(outputDrive(ctl), 500);

    advance(DEFAULT_WATCHDOG_TIMEOUT_MS - 1);
    CHECK_EQ(outputDrive(ctl), 500);
    advance(1);
    CHECK_EQ(outputDrive(ctl), 0);
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);
}

static void an_invalid_command_while_moving_stops_under_the_stop_policy() {
    g_now = 0;
    RoverController ctl(fakeMillis, 300, 50, INVALID_CMD_STOP);
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    CHECK_EQ(outputDrive(ctl), 500);

    CHECK(!sendFrame(ctl, CMD_MAX + 1, 0, MODE_TELEOP, false));
    CHECK(ctl.stopLatched());
    CHECK_EQ(outputDrive(ctl), 0);
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);

    CHECK(sendFrame(ctl, 500, 0, MODE_TELEOP, false));
    CHECK_EQ(outputDrive(ctl), 0);
    CHECK(sendFrame(ctl, 0, 0, MODE_TELEOP, false));
    CHECK(sendFrame(ctl, 500, 0, MODE_TELEOP, false));
    CHECK_EQ(outputDrive(ctl), 500);
}

static void a_wrong_source_frame_stops_under_the_stop_policy() {
    g_now = 0;
    RoverController ctl(fakeMillis, 300, 50, INVALID_CMD_STOP);
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    CHECK(!sendFrame(ctl, 900, 0, MODE_TELEOP, false, false, false,
                     INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(outputDrive(ctl), 0);
}

static void a_corrupt_frame_rides_out_under_either_policy() {
    const uint8_t policies[] = {INVALID_CMD_RIDE_OUT, INVALID_CMD_STOP};
    for (uint8_t policy : policies) {
        g_now = 0;
        RoverController ctl(fakeMillis, 300, 50, policy);
        sendControl(ctl, 500, 0, MODE_TELEOP, false);
        CHECK(!sendCorrupt(ctl, 500, MODE_TELEOP, false));
        CHECK_EQ(ctl.lastVerdict(), VERDICT_REJECT_CORRUPT);
        CHECK_EQ(outputDrive(ctl), 500);
    }
}

// A7: the rule table. Each row is one rule and runs as its own test.
struct RuleRow {
    const char* name;           // printed in the test output
    uint8_t     state;          // starting mode; TELEOP/AUTONOMOUS = already driving at 500
    bool        estop;          // turn the E-stop on first?
    Source      src;            // who sends the test command
    uint8_t     frame_mode;     // mode the command asks for
    int16_t     drive;          // drive value in the command
    bool        stop;           // STOP bit set?
    bool        abort;          // abort bit set?
    bool        corrupt;        // damage the CRC?
    uint8_t     expect_verdict; // the decision we expect
    int16_t     expect_drive;   // what the motors get after; 500 = rejected, old command kept
};

static const RuleRow RULES[] = {
    // name                                        state            estop  src           frame mode       drive  stop   abort  corrupt verdict                        drive
    {"rule: SAFE + drive",                         MODE_SAFE,       false, SRC_TELEOP,   MODE_SAFE,       600,   false, false, false,  VERDICT_REJECT_MODE_CONFLICT,  0},
    {"rule: SAFE + autonomy drive",                MODE_SAFE,       false, SRC_AUTONOMY, MODE_SAFE,       600,   false, false, false,  VERDICT_REJECT_MODE_CONFLICT,  0},
    {"rule: SAFE + autonomy grants itself AUTO",   MODE_SAFE,       false, SRC_AUTONOMY, MODE_AUTONOMOUS, 600,   false, false, false,  VERDICT_REJECT_MODE_REFUSED,   0},
    {"rule: SAFE + operator hands over AUTO",      MODE_SAFE,       false, SRC_TELEOP,   MODE_AUTONOMOUS, 0,     false, false, false,  VERDICT_ACCEPTED,              0},
    {"rule: SAFE + operator drives in AUTO",       MODE_SAFE,       false, SRC_TELEOP,   MODE_AUTONOMOUS, 600,   false, false, false,  VERDICT_REJECT_WRONG_SOURCE,   0},
    {"rule: TELEOP + teleop drive",                MODE_TELEOP,     false, SRC_TELEOP,   MODE_TELEOP,     600,   false, false, false,  VERDICT_ACCEPTED,              600},
    {"rule: TELEOP + autonomy drive",              MODE_TELEOP,     false, SRC_AUTONOMY, MODE_TELEOP,     600,   false, false, false,  VERDICT_REJECT_WRONG_SOURCE,   500},
    {"rule: TELEOP + autonomy takes AUTO",         MODE_TELEOP,     false, SRC_AUTONOMY, MODE_AUTONOMOUS, 600,   false, false, false,  VERDICT_REJECT_MODE_REFUSED,   500},
    {"rule: AUTONOMOUS + autonomy drive",          MODE_AUTONOMOUS, false, SRC_AUTONOMY, MODE_AUTONOMOUS, 600,   false, false, false,  VERDICT_ACCEPTED,              600},
    {"rule: AUTONOMOUS + teleop drive",            MODE_AUTONOMOUS, false, SRC_TELEOP,   MODE_AUTONOMOUS, 600,   false, false, false,  VERDICT_REJECT_WRONG_SOURCE,   500},
    {"rule: AUTONOMOUS + operator takes TELEOP",   MODE_AUTONOMOUS, false, SRC_TELEOP,   MODE_TELEOP,     0,     false, false, false,  VERDICT_ACCEPTED,              0},
    {"rule: TELEOP + STOP (operator)",             MODE_TELEOP,     false, SRC_TELEOP,   MODE_TELEOP,     600,   true,  false, false,  VERDICT_STOP,                  0},
    {"rule: TELEOP + STOP (autonomy)",             MODE_TELEOP,     false, SRC_AUTONOMY, MODE_TELEOP,     600,   true,  false, false,  VERDICT_STOP,                  0},
    {"rule: AUTONOMOUS + STOP (autonomy)",         MODE_AUTONOMOUS, false, SRC_AUTONOMY, MODE_AUTONOMOUS, 600,   true,  false, false,  VERDICT_STOP,                  0},
    {"rule: AUTONOMOUS + STOP (operator)",         MODE_AUTONOMOUS, false, SRC_TELEOP,   MODE_AUTONOMOUS, 600,   true,  false, false,  VERDICT_STOP,                  0},
    {"rule: AUTONOMOUS + abort (operator)",        MODE_AUTONOMOUS, false, SRC_TELEOP,   MODE_AUTONOMOUS, 600,   false, true,  false,  VERDICT_STOP,                  0},
    {"rule: TELEOP + STOP, corrupt",               MODE_TELEOP,     false, SRC_TELEOP,   MODE_TELEOP,     0,     true,  false, true,   VERDICT_REJECT_CORRUPT,        500},
    {"rule: TELEOP + STOP, out of range",          MODE_TELEOP,     false, SRC_TELEOP,   MODE_TELEOP,     5000,  true,  false, false,  VERDICT_REJECT_MALFORMED,      500},
    {"rule: TELEOP + drive out of range",          MODE_TELEOP,     false, SRC_TELEOP,   MODE_TELEOP,     5000,  false, false, false,  VERDICT_REJECT_MALFORMED,      500},
    {"rule: FAULT + drive",                        MODE_FAULT,      false, SRC_TELEOP,   MODE_TELEOP,     600,   false, false, false,  VERDICT_REJECT_MODE_REFUSED,   0},
    {"rule: E-stop while driving + drive",         MODE_TELEOP,     true,  SRC_TELEOP,   MODE_TELEOP,     600,   false, false, false,  VERDICT_REJECT_MODE_REFUSED,   0},
    {"rule: E-stop in SAFE + drive",               MODE_SAFE,       true,  SRC_TELEOP,   MODE_SAFE,       600,   false, false, false,  VERDICT_REJECT_MODE_CONFLICT,  0},
    {"rule: E-stop in SAFE + enter TELEOP, drive", MODE_SAFE,       true,  SRC_TELEOP,   MODE_TELEOP,     600,   false, false, false,  VERDICT_ACCEPTED,              0},
};

static void setUpRuleState(RoverController& ctl, uint8_t state) {
    sendFrame(ctl, 0, 0, MODE_SAFE, false);
    if (state == MODE_TELEOP || state == MODE_AUTONOMOUS) {
        grantMode(ctl, state);
        sendFrame(ctl, 500, 0, state, false);
    } else if (state == MODE_FAULT) {
        grantMode(ctl, MODE_TELEOP);
        sendFrame(ctl, 500, 0, MODE_TELEOP, false);
        advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    }
}

static void runRuleTable() {
    for (const RuleRow& row : RULES) {
        testing::beginTest(row.name);
        RoverController ctl = freshController();
        setUpRuleState(ctl, row.state);
        if (row.estop) ctl.updateFaults(FAULT_ESTOP_ACTIVE, CTRL_HEALTH_OK);

        if (row.corrupt) {
            sendCorrupt(ctl, row.drive, row.frame_mode, row.stop, row.src);
        } else {
            sendFrame(ctl, row.drive, 0, row.frame_mode, row.stop, row.abort,
                      false, INDICATOR_OFF, false, row.src);
        }
        CHECK_EQ(ctl.lastVerdict(), row.expect_verdict);
        CHECK_EQ(outputDrive(ctl), row.expect_drive);
        if (row.expect_drive != 0) CHECK(!ctl.effectiveStop());
        testing::endTest();
    }
}

int main() {
    std::printf("test_controller\n");
    RUN_TEST(boots_stopped_and_safe);
    RUN_TEST(valid_control_releases_the_stop);
    RUN_TEST(watchdog_trips_exactly_at_the_boundary);
    RUN_TEST(watchdog_is_correct_across_millis_wraparound);
    RUN_TEST(explicit_stop_flag_wins_in_any_mode);
    RUN_TEST(stop_latches_until_a_neutral_command);
    RUN_TEST(disabled_mode_forces_stop_at_full_throttle);
    RUN_TEST(undefined_mode_never_permits_motion);
    RUN_TEST(autonomy_abort_stops_an_autonomous_rover);
    RUN_TEST(autonomy_abort_does_not_stop_a_teleoperated_rover);
    RUN_TEST(return_request_is_carried_but_does_not_stop);
    RUN_TEST(wrong_dlc_does_not_refresh_the_watchdog);
    RUN_TEST(a_bad_crc_does_not_refresh_the_watchdog);
    RUN_TEST(foreign_can_id_does_not_refresh_the_watchdog);
    RUN_TEST(a_foreign_id_is_not_a_fault);
    RUN_TEST(out_of_range_command_never_moves_the_rover);
    RUN_TEST(out_of_range_command_does_not_refresh_the_watchdog);
    RUN_TEST(out_of_range_command_is_reported_as_a_protocol_error);
    RUN_TEST(contiguous_sequence_numbers_are_healthy);
    RUN_TEST(a_sequence_gap_is_detected_and_counted);
    RUN_TEST(the_sequence_wrap_is_not_a_gap);
    RUN_TEST(a_duplicate_sequence_number_is_flagged);
    RUN_TEST(a_sequence_gap_clears_when_the_stream_recovers);
    RUN_TEST(the_first_frame_is_never_a_gap);
    RUN_TEST(the_link_returns_to_ok_after_the_degraded_hold_expires);
    RUN_TEST(the_c2_link_is_never_derived_from_the_jetson_link);
    RUN_TEST(c2_loss_stops_a_teleoperated_rover_but_not_an_autonomous_one);
    RUN_TEST(the_c2_fault_is_raised_in_every_mode);
    RUN_TEST(a_stale_c2_bit_is_reported_as_unknown_not_as_ok_or_lost);
    RUN_TEST(state_echoes_the_mode_the_controller_believes_in);
    RUN_TEST(active_mode_follows_the_commanded_mode);
    RUN_TEST(comm_timeout_faults_a_moving_rover);
    RUN_TEST(comm_timeout_in_safe_is_reported_but_is_not_a_fault);
    RUN_TEST(fault_latches_until_safe_is_commanded);
    RUN_TEST(fault_cannot_be_left_straight_into_motion);
    RUN_TEST(a_timeout_latches_even_if_nobody_looked_in_between);
    RUN_TEST(the_latched_reason_outlives_the_condition);
    RUN_TEST(the_state_frame_reports_fault);
    RUN_TEST(c2_loss_faults_teleop_but_not_autonomy);
    RUN_TEST(a_pause_or_abort_is_not_a_fault);
    RUN_TEST(fault_cannot_be_commanded);
    RUN_TEST(stop_decision_is_unchanged_by_active_mode);
    RUN_TEST(the_indicator_follows_the_mode_the_controller_is_in);
    RUN_TEST(the_indicator_shows_fault_when_nothing_is_driving);
    RUN_TEST(a_non_stopping_fault_does_not_blank_the_indicator);
    RUN_TEST(power_reports_the_same_fault_word_the_controller_acts_on);
    RUN_TEST(drive_r_carries_the_command_age);
    RUN_TEST(age_is_unknown_before_the_first_command);
    RUN_TEST(telemetry_paces_at_the_configured_period);
    RUN_TEST(the_telemetry_sequence_advances_once_per_cycle);
    RUN_TEST(telemetry_does_not_burst_after_a_stall);
    RUN_TEST(telemetry_pacing_is_correct_across_wraparound);
    RUN_TEST(an_autonomy_command_is_refused_while_teleoperated);
    RUN_TEST(a_teleop_command_is_refused_while_autonomous);
    RUN_TEST(the_same_command_is_accepted_once_the_mode_matches);
    RUN_TEST(stop_overrides_an_authorised_autonomy_command);
    RUN_TEST(a_stop_from_the_wrong_source_still_stops_the_rover);
    RUN_TEST(an_abort_from_the_wrong_source_still_stops_the_rover);
    RUN_TEST(disabled_mode_has_no_owner_so_neither_source_mismatches);
    RUN_TEST(a_source_mismatch_is_held_then_expires);
    RUN_TEST(an_unauthorised_frame_does_not_feed_the_watchdog);
    RUN_TEST(an_arrival_claim_from_the_wrong_source_is_ignored);
    RUN_TEST(a_return_request_is_reported_but_changes_no_behaviour);
    RUN_TEST(a_return_request_outside_autonomous_is_not_active);
    RUN_TEST(a_return_request_from_the_operator_is_honoured);
    RUN_TEST(a_stale_return_request_is_not_reported);
    RUN_TEST(the_echo_acknowledges_the_sequence_number_it_acted_on);
    RUN_TEST(the_echo_reports_no_command_before_the_first_frame);
    RUN_TEST(the_echo_carries_the_reason_the_last_frame_was_rejected);
    RUN_TEST(the_echo_reports_a_refused_command_as_zero);
    RUN_TEST(the_echo_reports_an_active_return_request);
    RUN_TEST(outputs_are_nonzero_only_when_permitted_and_authorised);
    RUN_TEST(a_refused_command_is_not_a_stop);
    RUN_TEST(autonomy_cannot_put_itself_in_charge);
    RUN_TEST(autonomy_cannot_take_over_from_the_operator);
    RUN_TEST(a_refused_mode_change_does_not_drive_in_the_old_mode);
    RUN_TEST(the_operator_hands_over_and_autonomy_drives);
    RUN_TEST(the_operator_can_switch_between_the_motion_modes);
    RUN_TEST(either_source_can_stand_the_rover_down);
    RUN_TEST(teleop_is_not_entered_while_c2_is_lost);
    RUN_TEST(a_refusal_is_not_reported_once_it_is_stale);
    RUN_TEST(a_critical_fault_faults_a_moving_rover);
    RUN_TEST(a_drive_stage_fault_faults_a_moving_rover);
    RUN_TEST(safe_is_refused_while_the_fault_is_still_there);
    RUN_TEST(non_critical_faults_are_reported_but_do_not_stop_the_rover);
    RUN_TEST(a_critical_fault_in_safe_does_not_latch_but_blocks_driving);
    RUN_TEST(commanded_is_what_the_drive_stage_is_handed);
    RUN_TEST(measured_is_whatever_the_plant_last_reported);
    RUN_TEST(measured_follows_commanded_in_the_closed_loop);
    RUN_TEST(a_stalled_drivetrain_leaves_the_gap_open);
    RUN_TEST(a_channel_that_never_reported_is_unknown_not_zero);
    RUN_TEST(the_age_is_measured_from_the_last_update);
    RUN_TEST(a_frozen_channel_goes_stale_while_the_other_stays_fresh);
    RUN_TEST(staleness_trips_exactly_at_the_boundary);
    RUN_TEST(the_age_is_correct_across_millis_wraparound);
    RUN_TEST(a_very_old_reading_saturates_without_colliding_with_unknown);
    RUN_TEST(an_impossible_reading_is_invalid_but_not_stale);
    RUN_TEST(stale_and_invalid_reach_the_fault_word);
    RUN_TEST(stale_feedback_does_not_stop_the_rover);
    RUN_TEST(the_source_is_none_before_anybody_has_spoken);
    RUN_TEST(the_source_names_the_producer_that_is_driving);
    RUN_TEST(a_refused_source_does_not_displace_the_active_source);
    RUN_TEST(safe_has_no_owner_so_it_has_no_source);
    RUN_TEST(a_stale_source_is_not_reported);
    RUN_TEST(the_feedback_frame_carries_what_the_accessors_say);
    RUN_TEST(the_feedback_frame_round_trips_on_the_wire);
    RUN_TEST(the_arbiter_does_not_flap_between_two_producers);
    RUN_TEST(motion_in_safe_is_a_mode_conflict);
    RUN_TEST(an_invalid_command_does_not_replace_the_active_command);
    RUN_TEST(a_refused_mode_change_does_not_feed_the_watchdog);
    RUN_TEST(a_motion_command_in_safe_does_not_feed_the_watchdog);
    RUN_TEST(a_stop_feeds_the_watchdog);
    RUN_TEST(a_jetson_sending_only_refused_frames_is_alive_but_stale);
    RUN_TEST(entering_safe_clears_the_refusal_reports);
    RUN_TEST(stop_is_not_overridden_by_the_next_autonomy_command);
    RUN_TEST(the_rover_boots_with_stop_latched);
    RUN_TEST(an_abort_latches_like_a_stop_while_autonomous);
    RUN_TEST(only_the_operator_can_release_stop);
    RUN_TEST(a_handover_with_stop_set_keeps_autonomy_stopped);
    RUN_TEST(a_stop_must_pass_gates_one_and_two);
    RUN_TEST(stop_is_not_a_mode_change);
    RUN_TEST(a_stop_frame_still_obeys_the_mode_rules);
    RUN_TEST(an_invalid_command_while_moving_rides_out_to_the_timeout);
    RUN_TEST(an_invalid_command_while_moving_stops_under_the_stop_policy);
    RUN_TEST(a_wrong_source_frame_stops_under_the_stop_policy);
    RUN_TEST(a_corrupt_frame_rides_out_under_either_policy);
    runRuleTable();
    return testing::summary("test_controller");
}
