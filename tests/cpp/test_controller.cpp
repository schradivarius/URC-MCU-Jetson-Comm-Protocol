// test_controller.cpp -- safety state machine tests.
//
// Start a safety review here. Every test drives a FAKE CLOCK the test advances
// by hand, so watchdog timing is exact and deterministic rather than
// sleep-dependent -- a flaky safety test gets ignored, which is worse than no
// safety test.

#include "../../firmware/src/rover_controller.h"
#include "test_framework.h"

using namespace rover;

static uint32_t g_now = 0;
static uint32_t fakeMillis() { return g_now; }
static void advance(uint32_t ms) { g_now += ms; }   // uint32 wraps naturally

static RoverController freshController(uint32_t start_ms = 0) {
    g_now = start_ms;
    return RoverController(fakeMillis, 300, 50);
}

// Which producer a frame claims to come from. SRC_OWNER is "whichever one
// owns the mode being commanded", i.e. an ordinary authorised frame -- so a
// test that is not ABOUT authority does not have to think about it, and the
// ones that are say so loudly.
enum Source { SRC_OWNER = -1, SRC_TELEOP = 0, SRC_AUTONOMY = 1 };

static bool sourceBit(Source src, uint8_t mode) {
    if (src == SRC_OWNER) return mode == MODE_AUTONOMOUS;
    return src == SRC_AUTONOMY;
}

// Send ONE well-formed CONTROL frame, exactly as given. seq increments so the
// healthy path never trips the gap detector by accident. The mode-transition
// tests use this directly, because they are about what one frame may change.
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

// The operator granting a motion mode (v0.7): a stopped, operator-sourced
// frame asking for it. This is what the Jetson sends when C2 says "go
// autonomous", before the autonomy stack's own frames start arriving.
static void grantMode(RoverController& ctl, uint8_t mode, bool c2_lost = false) {
    sendFrame(ctl, 0, 0, mode, /*stop=*/true, false, false, INDICATOR_OFF,
              c2_lost, SRC_TELEOP);
}

// Send a CONTROL frame in `mode`, having the operator grant that mode first if
// the controller is not already in it. Same idea as SRC_OWNER: a test that is
// not ABOUT transitions does not have to think about them. The ones that are
// use sendFrame() and say so.
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

// --- boot and basic release -------------------------------------------------

static void boots_stopped_and_safe() {
    RoverController ctl = freshController();
    CHECK(ctl.effectiveStop());
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), CMD_AGE_UNKNOWN);
    CHECK_EQ(ctl.lastControl().mode, MODE_SAFE);
    // Standing by, not faulted (v0.7): FAULT is entered from a mode that could
    // move, and a booting rover is not one. The silent Jetson is still
    // reported -- as a comm timeout and a lost link, not as a mode.
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);
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

// --- watchdog ---------------------------------------------------------------

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
    // Arduino millis() is uint32 and wraps every ~49.7 days. This passes
    // because elapsed time is an unsigned SUBTRACTION, correct modulo 2^32. It
    // FAILS against a timestamp comparison (now >= then + timeout) -- verified
    // by building one deliberately.
    RoverController ctl = freshController(0xFFFFFF00u);
    sendControl(ctl, 500, 0, MODE_TELEOP, false);

    // CRITICAL pre-wrap check. A naive deadline (last + 300) has itself
    // wrapped to a tiny number here, so a huge pre-wrap `now` compares greater
    // and reports a timeout that has not happened. Without this assertion the
    // whole test passes against the buggy implementation.
    advance(80);
    CHECK(g_now > 0xFFFFFF00u);
    CHECK(!ctl.watchdogTripped());
    CHECK_EQ(ctl.cmdAgeMs(), 80);

    advance(219);                    // 299 total; clock now past the wrap
    CHECK(g_now < 0xFFFFFF00u);
    CHECK(!ctl.watchdogTripped());
    advance(1);
    CHECK(ctl.watchdogTripped());
}

// --- the stop triggers ------------------------------------------------------

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

static void stop_is_releasable_without_a_mode_change() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, true);
    CHECK(ctl.effectiveStop());
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    CHECK(!ctl.effectiveStop());
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

// --- autonomy abort and return request --------------------------------------

static void autonomy_abort_stops_an_autonomous_rover() {
    RoverController ctl = freshController();
    sendControl(ctl, 800, 0, MODE_AUTONOMOUS, false, /*abort=*/true);
    CHECK(ctl.autonomyAbort());
    CHECK(ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
}

static void autonomy_abort_does_not_stop_a_teleoperated_rover() {
    // In TELEOP the operator is already driving; there is no autonomous task
    // to abort, and stopping would surprise them mid-manoeuvre.
    RoverController ctl = freshController();
    sendControl(ctl, 800, 0, MODE_TELEOP, false, /*abort=*/true);
    CHECK(ctl.autonomyAbort());
    CHECK(!ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 800);
}

static void return_request_is_carried_but_does_not_stop() {
    // A behaviour request for a higher layer, not a safety input.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, /*ret=*/true);
    CHECK(ctl.returnRequest());
    CHECK(!ctl.effectiveStop());
}

// --- what must NOT refresh the watchdog -------------------------------------

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
    // The app-layer CRC's whole purpose: a frame corrupted in the software
    // path is rejected, and a rejected frame must not keep the rover alive.
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
        buf[0] ^= 0xFF;                       // corrupt AFTER sealing
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
    CHECK_EQ(ctl.lastControl().drive_cmd, 500);     // not overwritten
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
    // The whole point of rejecting rather than clamping: a sender that
    // disagrees with us about the scale must not be able to keep the rover
    // alive on commands we never understood.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 0, MODE_TELEOP, false));
    advance(290);
    CHECK(!ctl.watchdogTripped());
    for (int i = 0; i < 5; ++i) sendControl(ctl, 30000, 0, MODE_TELEOP, false);
    advance(20);
    CHECK(ctl.watchdogTripped());
    CHECK_EQ(ctl.lastControl().drive_cmd, 500);     // not overwritten
}

static void out_of_range_command_is_reported_as_a_protocol_error() {
    // A stop must be explainable. A rover that halts while telemetry reads
    // "no faults" is its own hazard.
    RoverController ctl = freshController();
    sendControl(ctl, 0, CMD_MIN - 1, MODE_TELEOP, false);
    CHECK(ctl.faultWord() & FAULT_PROTOCOL_ERROR);
    CHECK(!(ctl.faultWord() & FAULT_CRC_ERROR));    // not a corruption fault
    CHECK(sendControl(ctl, 0, CMD_MIN, MODE_TELEOP, false));   // full scale is fine
    CHECK(!(ctl.faultWord() & FAULT_PROTOCOL_ERROR));          // and self-clears
}

static void a_foreign_id_is_not_a_fault() {
    // Other traffic on a shared bus is normal. Flagging it would make the
    // protocol-error bit permanently on and therefore useless.
    RoverController ctl = freshController();
    sendControl(ctl, 400, 0, MODE_TELEOP, false);
    uint8_t buf[8] = {0};
    ctl.ingestFrame(0x321, buf, 8);
    CHECK(!(ctl.faultWord() & FAULT_PROTOCOL_ERROR));
    CHECK(!(ctl.faultWord() & FAULT_CRC_ERROR));
}

// --- sequence numbers -------------------------------------------------------

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
    sendWithSeq(ctl, 14);                // 11, 12, 13 missing
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    CHECK_EQ(ctl.seqGapsSeen(), 1u);
    CHECK_EQ(ctl.framesLostEstimate(), 3u);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_DEGRADED);
}

static void the_sequence_wrap_is_not_a_gap() {
    // 255 -> 0 is a delta of 1. Treating it as a 255-frame loss would raise a
    // false fault once every 256 frames, i.e. every 13 seconds at 20 Hz.
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
    sendWithSeq(ctl, 5);                 // delta 0
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    CHECK_EQ(ctl.framesLostEstimate(), 0u);   // nothing lost, just repeated
}

static void a_sequence_gap_clears_when_the_stream_recovers() {
    RoverController ctl = freshController();
    sendWithSeq(ctl, 1);
    sendWithSeq(ctl, 9);
    CHECK(ctl.faultWord() & FAULT_SEQ_GAP);
    sendWithSeq(ctl, 10);                // contiguous again
    CHECK(!(ctl.faultWord() & FAULT_SEQ_GAP));
}

static void the_first_frame_is_never_a_gap() {
    // There is no previous sequence number to compare against at boot.
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
    // Keep the command watchdog fed while the degraded hold runs out.
    for (uint32_t t = 0; t < LINK_DEGRADED_HOLD_MS + 100; t += 50) {
        advance(50);
        sendWithSeq(ctl, static_cast<uint8_t>(6 + t / 50));
    }
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
}

// --- link separation: the point of the whole exercise -----------------------

static void the_c2_link_is_never_derived_from_the_jetson_link() {
    // The 2027 autonomy course deliberately includes areas with no C2
    // line-of-sight while onboard autonomy keeps working. Collapsing the two
    // links would stop the rover exactly where it is supposed to keep going.
    RoverController ctl = freshController();

    // Before any frame: honestly "nobody told us", NOT a misleading OK, and
    // not derived from the boot default of the c2_lost field either.
    CHECK_EQ(ctl.c2LinkState(), LINK_NOT_REPORTED);

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, /*c2_lost=*/false);
    TelemetryState st = ctl.buildState();
    CHECK_EQ(st.jetson_link, LINK_OK);
    CHECK_EQ(st.c2_link, LINK_OK);

    // A reported C2 loss must not change the Jetson link, and must not stop a
    // healthy autonomous rover.
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, /*c2_lost=*/true);
    st = ctl.buildState();
    CHECK_EQ(st.c2_link, LINK_LOST);
    CHECK_EQ(st.jetson_link, LINK_OK);
    CHECK(!ctl.effectiveStop());
}

static void c2_loss_stops_a_teleoperated_rover_but_not_an_autonomous_one() {
    // The asymmetry is the whole point. In TELEOP the operator's commands can
    // no longer arrive, so nobody is driving. In AUTONOMOUS the Jetson IS
    // driving, and is provably alive or the watchdog would have stopped us.
    RoverController ctl = freshController();

    sendControl(ctl, 800, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, /*c2_lost=*/true);
    CHECK(!ctl.effectiveStop());
    int16_t drive = -1, steer = -1;
    ctl.commandedOutputs(drive, steer);
    CHECK_EQ(drive, 800);

    RoverController tele = freshController();
    sendControl(tele, 800, 0, MODE_TELEOP, false, false, false, INDICATOR_TELEOP);
    sendControl(tele, 800, 0, MODE_TELEOP, false, false, false,
                INDICATOR_TELEOP, /*c2_lost=*/true);
    CHECK(tele.effectiveStop());
    tele.commandedOutputs(drive, steer);
    CHECK_EQ(drive, 0);

    // And since v0.7 it does NOT release when the Jetson says the link is
    // back: FAULT latches, and only a SAFE command leaves it.
    sendControl(tele, 800, 0, MODE_TELEOP, false, false, false,
                INDICATOR_TELEOP, /*c2_lost=*/false);
    CHECK(tele.effectiveStop());
    CHECK_EQ(tele.activeMode(), MODE_FAULT);
}

static void the_c2_fault_is_raised_in_every_mode() {
    // It only stops the rover in TELEOP, but during an autonomous run it is
    // the single sign that the rover is out of contact, so it must be
    // reported whether or not it changes the driving.
    const uint8_t modes[] = {MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS};
    for (uint8_t mode : modes) {
        RoverController ctl = freshController();
        sendControl(ctl, 0, 0, mode, false, false, false, INDICATOR_OFF,
                    /*c2_lost=*/true);
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) != 0);
        sendControl(ctl, 0, 0, mode, false, false, false, INDICATOR_OFF,
                    /*c2_lost=*/false);
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) == 0);
    }
}

static void a_stale_c2_bit_is_reported_as_unknown_not_as_ok_or_lost() {
    // The controller only knows c2_lost from the LAST CONTROL frame. Once the
    // Jetson goes silent that value is stale: a clear bit ten seconds ago does
    // not mean C2 is fine now, and a set bit does not mean it is still down.
    // Reporting either would be worse than admitting we do not know.
    for (int lost = 0; lost <= 1; ++lost) {
        RoverController ctl = freshController();
        sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                    INDICATOR_AUTONOMOUS, /*c2_lost=*/lost != 0);
        CHECK_EQ(ctl.c2LinkState(), lost ? LINK_LOST : LINK_OK);

        advance(400);                    // watchdog trips; the bit goes stale
        CHECK_EQ(ctl.c2LinkState(), LINK_NOT_REPORTED);
        CHECK_EQ(ctl.buildState().c2_link, LINK_NOT_REPORTED);
        // And the fault bit goes with it, rather than latching a stale claim.
        CHECK((ctl.faultWord() & FAULT_C2_LINK_LOST) == 0);
        CHECK((ctl.faultWord() & FAULT_COMM_TIMEOUT) != 0);
    }
}

// --- telemetry --------------------------------------------------------------

static void state_echoes_the_mode_the_controller_believes_in() {
    // Without the echo, a dropped or misread mode change is invisible from
    // outside the rover.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false);
    CHECK_EQ(ctl.buildState().mode, MODE_AUTONOMOUS);
    sendControl(ctl, 0, 0, MODE_SAFE, false);
    CHECK_EQ(ctl.buildState().mode, MODE_SAFE);
}

// --- active mode: what the rover is actually executing ----------------------

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
        CHECK_EQ(ctl.activeMode(), mode);          // one ms early: still fine
        advance(1);
        CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    }
}

static void comm_timeout_in_safe_is_reported_but_is_not_a_fault() {
    // The spec enters FAULT from TELEOP or AUTONOMOUS. A rover in SAFE is not
    // moving and was not going to, so a silent Jetson there is a link problem
    // to report, not a reason to latch the rover into a state it then has to
    // be talked out of.
    RoverController ctl = freshController();
    sendControl(ctl, 0, 0, MODE_SAFE, false);
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);
    CHECK_EQ(ctl.jetsonLinkState(), LINK_LOST);
    CHECK(ctl.effectiveStop());
}

static void fault_latches_until_safe_is_commanded() {
    // v0.7: getting the link back does not resume driving. The rover stopped
    // for a reason, and someone has to acknowledge it -- by commanding SAFE --
    // before it may be given a motion mode again.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);

    // The link is back and the Jetson carries on as if nothing happened.
    CHECK(sendFrame(ctl, 500, 0, MODE_TELEOP, false));
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK(ctl.modeRefused());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    // The reason is still there, although the link itself is fine now.
    CHECK(!ctl.watchdogTripped());
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);

    // SAFE is the way out, and it clears the latched reason.
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false));
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK_EQ(ctl.latchedFaults(), 0);
    CHECK(!(ctl.faultWord() & FAULT_COMM_TIMEOUT));

    // And from SAFE the operator can drive again.
    CHECK(sendFrame(ctl, 500, 0, MODE_TELEOP, false));
    ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 500);
}

static void fault_cannot_be_left_straight_into_motion() {
    // FAULT -> TELEOP and FAULT -> AUTONOMOUS are refused even from the
    // operator: SAFE first, every time.
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
    // Nothing queries the controller between the trip and the next frame. The
    // frame must still find the rover faulted: ingestFrame() latches BEFORE it
    // refreshes the watchdog, or the evidence would be overwritten.
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
              /*c2_lost=*/true);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK_EQ(ctl.latchedFaults(), FAULT_C2_LINK_LOST);

    // C2 is back. The bit stays, so the operator can see WHY the rover is
    // sitting in FAULT; otherwise it would read as a fault with no cause.
    sendFrame(ctl, 500, 0, MODE_TELEOP, false);
    CHECK_EQ(ctl.c2LinkState(), LINK_OK);
    CHECK(ctl.faultWord() & FAULT_C2_LINK_LOST);

    sendFrame(ctl, 0, 0, MODE_SAFE, false);
    CHECK(!(ctl.faultWord() & FAULT_C2_LINK_LOST));
}

static void the_state_frame_reports_fault() {
    // v0.7: TELEM_STATE.mode is the mode the controller is IN. Before, it
    // echoed the request, and a faulted rover reported the mode it had been
    // told rather than the one it was executing.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false);
    advance(DEFAULT_WATCHDOG_TIMEOUT_MS);
    CHECK_EQ(ctl.buildState().mode, MODE_FAULT);
}

static void c2_loss_faults_teleop_but_not_autonomy() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false, false, false, INDICATOR_TELEOP);
    sendControl(ctl, 500, 0, MODE_TELEOP, false, false, false,
                INDICATOR_TELEOP, /*c2_lost=*/true);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK(ctl.effectiveStop());

    RoverController aut = freshController();
    sendControl(aut, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS, /*c2_lost=*/true);
    CHECK_EQ(aut.activeMode(), MODE_AUTONOMOUS);
    CHECK(!aut.effectiveStop());
}

static void a_pause_or_abort_is_not_a_fault() {
    // Commanded stops: the rover is still in the mode it was told, just held.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, /*stop=*/true);
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, /*abort=*/true);
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
}

static void fault_cannot_be_commanded() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    CHECK(!sendControl(ctl, 500, 0, MODE_FAULT, false));
    CHECK(ctl.faultWord() & FAULT_PROTOCOL_ERROR);
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);       // last valid command, until the watchdog
}

static void stop_decision_is_unchanged_by_active_mode() {
    // effectiveStop() is now written in terms of activeMode(). Prove it gives
    // exactly the answer the previous explicit formula gave, in every
    // combination that formula looked at, so the refactor cannot have moved
    // the line between stopped and driving.
    const uint8_t modes[] = {MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS};
    const uint32_t ages[] = {0, DEFAULT_WATCHDOG_TIMEOUT_MS - 1,
                             DEFAULT_WATCHDOG_TIMEOUT_MS};
    for (int booted = 0; booted <= 1; ++booted)
    for (uint8_t mode : modes)
    for (int stop = 0; stop <= 1; ++stop)
    for (int abort = 0; abort <= 1; ++abort)
    for (int c2 = 0; c2 <= 1; ++c2)
    for (uint32_t age : ages) {
        RoverController ctl = freshController(0xFFFFFF80u);   // straddles the wrap
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
    // URC scores this light on what the rover is ACTUALLY doing, so the
    // controller's own mode wins over the Jetson's request.
    RoverController ctl = freshController();

    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.buildState().indicator_state, INDICATOR_AUTONOMOUS);

    // Only ARRIVED is honoured in AUTONOMOUS; it is the one thing the
    // controller cannot know for itself.
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_ARRIVED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_ARRIVED);

    // A human-driven rover never shows red or green, whatever is asked for.
    sendControl(ctl, 500, 0, MODE_TELEOP, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_TELEOP);
    sendControl(ctl, 500, 0, MODE_TELEOP, false, false, false,
                INDICATOR_ARRIVED);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_TELEOP);

    // SAFE is a legitimate idle state, not a fault.
    sendControl(ctl, 0, 0, MODE_SAFE, false, false, false,
                INDICATOR_AUTONOMOUS);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_OFF);
}

static void the_indicator_shows_fault_when_nothing_is_driving() {
    // A rover nobody can reach must not cheerfully display "autonomous" or
    // "arrived" -- and OFF would read as a legitimate idle state, so FAULT.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, false,
                INDICATOR_AUTONOMOUS);
    advance(400);                        // watchdog trips -> COMM_TIMEOUT
    CHECK_EQ(ctl.indicatorState(), INDICATOR_FAULT);
    CHECK_EQ(ctl.buildState().indicator_state, INDICATOR_FAULT);

    // C2 lost in TELEOP is the other case where nobody is driving.
    RoverController ctl2 = freshController();
    sendControl(ctl2, 500, 0, MODE_TELEOP, false, false, false, INDICATOR_TELEOP);
    sendControl(ctl2, 500, 0, MODE_TELEOP, false, false, false,
                INDICATOR_TELEOP, /*c2_lost=*/true);
    CHECK_EQ(ctl2.indicatorState(), INDICATOR_FAULT);
}

static void a_non_stopping_fault_does_not_blank_the_indicator() {
    // URC requires red while autonomous. An undervoltage warning or a single
    // lost frame must not take the light away from what the rover is doing;
    // those belong in fault_status, which is where an operator reads them.
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
    CHECK(pw.fault_status & FAULT_COMM_TIMEOUT);    // no command yet
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

// --- telemetry pacing and the shared sequence number ------------------------

static void telemetry_paces_at_the_configured_period() {
    RoverController ctl = freshController();
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());
    advance(50);
    CHECK(ctl.telemetryDue());
}

static void the_telemetry_sequence_advances_once_per_cycle() {
    // All four frames of a cycle must share one sequence number -- that is how
    // the Jetson tells a coherent snapshot from one torn across two cycles.
    RoverController ctl = freshController();
    CHECK(ctl.telemetryDue());
    const uint8_t first = ctl.telemetrySeq();
    CHECK_EQ(ctl.telemetrySeq(), first);      // reading it does not advance it
    CHECK_EQ(ctl.telemetrySeq(), first);
    advance(50);
    CHECK(ctl.telemetryDue());
    CHECK_EQ(seqDelta(first, ctl.telemetrySeq()), 1);
}

static void telemetry_does_not_burst_after_a_stall() {
    RoverController ctl = freshController();
    ctl.telemetryDue();
    advance(5000);                       // a 5 second stall
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());          // exactly one, not a hundred
}

static void telemetry_pacing_is_correct_across_wraparound() {
    RoverController ctl = freshController(0xFFFFFFF0u);
    CHECK(ctl.telemetryDue());
    CHECK(!ctl.telemetryDue());
    advance(50);
    CHECK(g_now < 0xFFFFFFF0u);
    CHECK(ctl.telemetryDue());
}


// --- command authority: the Task 4 mux --------------------------------------
//
// The rule under test: a producer may command MOTION only in the mode it
// owns. The autonomy stack owns AUTONOMOUS, the operator owns TELEOP. A frame
// from the other one is still a VALID frame -- it keeps the link alive and
// its mode echo and safety flags are honoured -- it just may not drive.

static void an_autonomy_command_is_refused_while_teleoperated() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 600, 200, MODE_TELEOP, false, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    // Accepted as a frame...
    CHECK_EQ(ctl.lastDecodeResult(), DECODE_OK);
    CHECK_EQ(ctl.controlFramesAccepted(), 2u);   // the operator's grant, then this
    CHECK(!ctl.watchdogTripped());
    // ...but not acted on, and NOT by pretending the rover is stopped.
    CHECK(!ctl.commandAuthorized());
    CHECK(!ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_WRONG_SOURCE);
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
}

static void a_teleop_command_is_refused_while_autonomous() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 600, 200, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(!ctl.commandAuthorized());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_WRONG_SOURCE);
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);
}

static void the_same_command_is_accepted_once_the_mode_matches() {
    // The whole Task 4 demonstration in one test: identical command, refused
    // in one mode and applied in the other, with nothing else changed.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 600, 200, MODE_TELEOP, false, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);

    CHECK(sendControl(ctl, 600, 200, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK(ctl.commandAuthorized());
    ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 600); CHECK_EQ(s, 200);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_APPLIED);
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));
}

static void stop_overrides_an_authorised_autonomy_command() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 900, 0, MODE_AUTONOMOUS, true, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK(ctl.commandAuthorized());        // authority is fine...
    CHECK(ctl.effectiveStop());            // ...and irrelevant under a stop
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
}

static void a_stop_from_the_wrong_source_still_stops_the_rover() {
    // Safety flags are never gated by authority. An operator who can see the
    // rover must be able to stop it even while autonomy owns the mode.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 900, 0, MODE_AUTONOMOUS, true, false, false,
                      INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(!ctl.commandAuthorized());
    CHECK(ctl.effectiveStop());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);   // stop outranks source
}

static void an_abort_from_the_wrong_source_still_stops_the_rover() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 900, 0, MODE_AUTONOMOUS, false, /*abort=*/true,
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
        // Nothing moves regardless -- that is the mode, not the authority.
        CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
    }
}

static void a_stale_source_mismatch_is_not_reported() {
    // Same rule as the stale C2 bit: once the watchdog has tripped, the
    // controller cannot vouch for anything the last frame said, including who
    // sent it. Reporting a mismatch on top of a comm timeout would send an
    // operator chasing the wrong bug.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(ctl.sourceMismatch());
    CHECK(ctl.faultWord() & FAULT_CMD_SOURCE);

    advance(400);                               // past the 300 ms watchdog
    CHECK(ctl.watchdogTripped());
    CHECK(!ctl.sourceMismatch());
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));
    CHECK(ctl.faultWord() & FAULT_COMM_TIMEOUT);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
}

static void an_unauthorised_frame_still_feeds_the_watchdog() {
    // It is a valid frame from a live Jetson. Treating it as silence would
    // turn a mux bug into a comm timeout and hide the real cause.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_TELEOP));
    advance(200);
    CHECK(sendControl(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_TELEOP));
    advance(200);
    CHECK(!ctl.watchdogTripped());
    CHECK_EQ(ctl.jetsonLinkState(), LINK_OK);
}

static void an_arrival_claim_from_the_wrong_source_is_ignored() {
    // Flashing green is worth 25 points under rule 1.e.xvii. A teleop-path
    // frame asserting it while the rover is autonomous is the one mix-up that
    // could light it on a rover that has arrived nowhere.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_ARRIVED, false, SRC_TELEOP));
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);

    CHECK(sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_ARRIVED, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.indicatorState(), INDICATOR_ARRIVED);
}

// --- return request ---------------------------------------------------------

static void a_return_request_is_reported_but_changes_no_behaviour() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false, false, /*ret=*/true,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK(ctl.returnRequestActive());
    CHECK(ctl.returnRequest());
    // Deliberately NOT a stop: driving back is the autonomy stack's job, and
    // PROTOCOL.md section 9 leaves the semantics as a team decision.
    CHECK(!ctl.effectiveStop());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 500);
    CHECK_EQ(ctl.indicatorState(), INDICATOR_AUTONOMOUS);   // still red
}

static void a_return_request_outside_autonomous_is_not_active() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 500, 0, MODE_TELEOP, false, false, /*ret=*/true));
    CHECK(ctl.returnRequest());            // the bit arrived...
    CHECK(!ctl.returnRequestActive());     // ...but an autonomous return means
                                           // nothing while a human is driving
}

static void a_return_request_from_the_operator_is_honoured() {
    // Rule 1.e.xi makes signalling the return the OPERATOR's action, so it
    // must survive arriving from the teleop path during an autonomous run.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, /*ret=*/true,
                      INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(!ctl.commandAuthorized());
    CHECK(ctl.returnRequestActive());
}

static void a_stale_return_request_is_not_reported() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 0, 0, MODE_AUTONOMOUS, false, false, /*ret=*/true,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK(ctl.returnRequestActive());
    advance(400);
    CHECK(!ctl.returnRequestActive());
}

// --- the acknowledgement frame ---------------------------------------------

static void the_echo_acknowledges_the_sequence_number_it_acted_on() {
    RoverController ctl = freshController();
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
    // A frame can be rejected while an older accepted command is still being
    // applied, so the decode reason is its own field rather than a verdict.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 400, 0, MODE_TELEOP, false));
    uint8_t buf[8] = {0};
    ControlMsg m;
    m.drive_cmd = 400; m.steer_cmd = 0; m.mode = MODE_TELEOP;
    m.stop = false; m.autonomy_abort = false; m.return_request = false;
    m.c2_lost = false; m.from_autonomy = false;
    m.indicator_request = INDICATOR_OFF; m.seq = ++g_seq;
    encodeControl(m, buf);
    buf[0] ^= 0xFF;                               // corrupt after the CRC
    CHECK(!ctl.ingestFrame(CAN_ID_CONTROL, buf, FRAME_DLC));

    TelemetryCmdEcho e = ctl.buildCmdEcho(400, 0);
    CHECK_EQ(decodeResultOf(e.status), DECODE_BAD_CRC);
    CHECK_EQ(cmdStatusOf(e.status), CMD_STATUS_APPLIED);   // the old one stands
    CHECK_EQ(e.drive_applied, 400);
}

static void the_echo_reports_a_refused_command_as_zero() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 800, 300, MODE_TELEOP, false, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    int16_t d, s; ctl.commandedOutputs(d, s);
    TelemetryCmdEcho e = ctl.buildCmdEcho(d, s);
    CHECK_EQ(e.drive_applied, 0);
    CHECK_EQ(e.steer_applied, 0);
    CHECK_EQ(cmdStatusOf(e.status), CMD_STATUS_WRONG_SOURCE);
}

static void the_echo_reports_an_active_return_request() {
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 300, 0, MODE_AUTONOMOUS, false, false, /*ret=*/true,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    TelemetryCmdEcho e = ctl.buildCmdEcho(300, 0);
    CHECK(returnActiveOf(e.status));
}

// --- the invariant, swept exhaustively --------------------------------------

static void outputs_are_nonzero_only_when_permitted_and_authorised() {
    // Every reachable combination of the fields that can gate motion, times
    // "has the watchdog tripped". The invariant is the whole safety claim in
    // one line: a non-zero output requires BOTH that the rover may move and
    // that this command is its to obey.
    //
    // Written as a sweep rather than as cases because the gating conditions
    // now interact -- mode, source, stop, abort, C2 and the watchdog -- and a
    // future edit that gets one pairing wrong would otherwise need somebody to
    // have thought of that exact pairing in advance.
    int moved = 0, refused = 0, stopped_count = 0;
    for (int mode = 0; mode <= 3; ++mode) {
        for (int src = 0; src <= 1; ++src) {
            for (int stop = 0; stop <= 1; ++stop) {
                for (int abort = 0; abort <= 1; ++abort) {
                    for (int c2 = 0; c2 <= 1; ++c2) {
                        for (int stale = 0; stale <= 1; ++stale) {
                            RoverController ctl = freshController();
                            const bool accepted = sendControl(
                                ctl, 700, 300, static_cast<uint8_t>(mode),
                                stop != 0, abort != 0, false, INDICATOR_OFF,
                                c2 != 0, src ? SRC_AUTONOMY : SRC_TELEOP);
                            // mode 3 is undefined, so the frame is rejected
                            // and the rover is left in its boot state.
                            CHECK_EQ(accepted, mode <= 2);
                            if (stale) advance(400);

                            int16_t d, s;
                            ctl.commandedOutputs(d, s);
                            const bool permitted = !ctl.effectiveStop();
                            const bool authorised = ctl.commandAuthorized();

                            if (d != 0 || s != 0) {
                                CHECK(permitted);
                                CHECK(authorised);
                                CHECK_EQ(ctl.commandStatus(), CMD_STATUS_APPLIED);
                                CHECK_EQ(d, 700); CHECK_EQ(s, 300);
                                moved++;
                            } else if (!permitted) {
                                // NO_COMMAND rather than STOPPED when the
                                // frame was never accepted at all -- an
                                // undefined mode leaves the controller in its
                                // boot state, and "nobody has spoken to me" is
                                // the more useful of the two things to say.
                                //
                                // MODE_REFUSED when the frame asked for a mode
                                // the rover never entered: TELEOP while C2 is
                                // reported lost.
                                CHECK_EQ(ctl.commandStatus(),
                                         !accepted        ? CMD_STATUS_NO_COMMAND
                                         : ctl.modeRefused() ? CMD_STATUS_MODE_REFUSED
                                                             : CMD_STATUS_STOPPED);
                                stopped_count++;
                            } else {
                                CHECK(!authorised);
                                CHECK_EQ(ctl.commandStatus(), CMD_STATUS_WRONG_SOURCE);
                                refused++;
                            }

                            // The fault bit must agree with the verdict, and
                            // must never be raised on a stale frame.
                            const bool bit = (ctl.faultWord() & FAULT_CMD_SOURCE) != 0;
                            CHECK_EQ(bit, ctl.sourceMismatch());
                            if (stale) CHECK(!bit);
                        }
                    }
                }
            }
        }
    }
    // All three outcomes have to be reachable, or the sweep proved nothing by
    // never leaving one branch.
    CHECK(moved > 0);
    CHECK(refused > 0);
    CHECK(stopped_count > 0);
}

static void a_refused_command_is_not_a_stop() {
    // The distinction the whole design rests on: WRONG_SOURCE leaves the rover
    // able to move, so the drive stage stays energized at neutral and an
    // authorised command takes effect on the very next frame rather than after
    // a recovery. Reporting it as a stop would lose that.
    RoverController ctl = freshController();
    CHECK(sendControl(ctl, 700, 300, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_TELEOP));
    CHECK(!ctl.effectiveStop());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_WRONG_SOURCE);

    CHECK(sendControl(ctl, 700, 300, MODE_AUTONOMOUS, false, false, false,
                      INDICATOR_OFF, false, SRC_AUTONOMY));
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 700); CHECK_EQ(s, 300);
}

// --- mode transitions (v0.7) ------------------------------------------------
// The mode is the controller's state, and a frame only REQUESTS a change.
// These use sendFrame() rather than sendControl(), because they are about
// exactly what one frame is and is not allowed to change.

static void autonomy_cannot_put_itself_in_charge() {
    // The spec's "explicit C2 operator command" to enter AUTONOMOUS, enforced
    // where it cannot be bypassed by a Jetson-side bug.
    RoverController ctl = freshController();
    CHECK(sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                    INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
    CHECK(ctl.modeRefused());
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);
    CHECK_EQ(ctl.buildState().mode, MODE_SAFE);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    // A refused transition is its own verdict, not a source mismatch.
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));
    // And it is a valid frame from a live Jetson, so it feeds the watchdog.
    CHECK(!ctl.watchdogTripped());
}

static void autonomy_cannot_take_over_from_the_operator() {
    RoverController ctl = freshController();
    sendFrame(ctl, 300, 0, MODE_TELEOP, false);
    CHECK(sendFrame(ctl, 600, 0, MODE_AUTONOMOUS, false, false, false,
                    INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);
}

static void a_refused_mode_change_does_not_drive_in_the_old_mode() {
    // The trap this closes: autonomy asks for TELEOP while the rover is
    // AUTONOMOUS. The change is refused, so the rover is still AUTONOMOUS, and
    // the frame IS from autonomy -- but it was written for a different mode,
    // and obeying it would act on a command its sender never meant for here.
    RoverController ctl = freshController();
    grantMode(ctl, MODE_AUTONOMOUS);
    CHECK(sendFrame(ctl, 600, 0, MODE_TELEOP, false, false, false,
                    INDICATOR_OFF, false, SRC_AUTONOMY));
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
    CHECK(!ctl.commandAuthorized());
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 0); CHECK_EQ(s, 0);
    // Refused, not stopped: the next agreeing frame drives straight away.
    CHECK(!ctl.effectiveStop());
}

static void the_operator_hands_over_and_autonomy_drives() {
    // The whole handover: the operator grants AUTONOMOUS with a stopped
    // frame, then the autonomy stack's frames drive.
    RoverController ctl = freshController();
    grantMode(ctl, MODE_AUTONOMOUS);
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_STOPPED);
    // The grant is the operator's frame in autonomy's mode, but it asks for
    // nothing to move, so it is not a source mismatch and raises no alarm.
    CHECK(!(ctl.faultWord() & FAULT_CMD_SOURCE));

    CHECK(sendFrame(ctl, 600, 100, MODE_AUTONOMOUS, false));
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_APPLIED);
    int16_t d, s; ctl.commandedOutputs(d, s);
    CHECK_EQ(d, 600); CHECK_EQ(s, 100);
}

static void the_operator_can_switch_between_the_motion_modes() {
    // The spec lets TELEOP and AUTONOMOUS go to each other directly, as long
    // as the operator is the one asking.
    RoverController ctl = freshController();
    sendFrame(ctl, 0, 0, MODE_TELEOP, false);
    grantMode(ctl, MODE_AUTONOMOUS);
    CHECK_EQ(ctl.activeMode(), MODE_AUTONOMOUS);
    sendFrame(ctl, 0, 0, MODE_TELEOP, false);
    CHECK_EQ(ctl.activeMode(), MODE_TELEOP);
}

static void either_source_can_stand_the_rover_down() {
    // Anyone may make the rover safer. Same principle as the stop flag.
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
    // It would only fault on the same frame. Refusing keeps the rover where
    // it was, which is a better answer to "the operator cannot reach me".
    RoverController ctl = freshController();
    sendFrame(ctl, 500, 0, MODE_TELEOP, false, false, false, INDICATOR_OFF,
              /*c2_lost=*/true);
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

// --- critical hardware faults (v0.7) -----------------------------------------
// updateFaults() is how the firmware tells the controller what the sensors
// and the drive stage see. Which bits stop the rover is FAULT_CRITICAL_MASK.

static void a_critical_fault_faults_a_moving_rover() {
    const uint16_t critical[] = {FAULT_OVER_CURRENT, FAULT_ESTOP_ACTIVE};
    for (uint16_t bit : critical) {
        RoverController ctl = freshController();
        sendControl(ctl, 500, 0, MODE_TELEOP, false);
        ctl.updateFaults(bit, CTRL_HEALTH_OK);
        CHECK_EQ(ctl.activeMode(), MODE_FAULT);
        CHECK(ctl.effectiveStop());
        CHECK_EQ(ctl.latchedFaults(), bit);

        // The condition clears; the rover stays put until SAFE.
        ctl.updateFaults(0, CTRL_HEALTH_OK);
        CHECK_EQ(ctl.activeMode(), MODE_FAULT);
        CHECK(ctl.faultWord() & bit);
    }
}

static void a_drive_stage_fault_faults_a_moving_rover() {
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_AUTONOMOUS, false);
    ctl.updateFaults(0, CTRL_HEALTH_FAULT);
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
}

static void safe_is_refused_while_the_fault_is_still_there() {
    // "Validate system checks": FAULT -> SAFE is an acknowledgement that the
    // problem has been dealt with, and it has not been while it is present.
    RoverController ctl = freshController();
    sendControl(ctl, 500, 0, MODE_TELEOP, false);
    ctl.updateFaults(FAULT_OVER_CURRENT, CTRL_HEALTH_OK);

    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false));
    CHECK_EQ(ctl.activeMode(), MODE_FAULT);
    CHECK_EQ(ctl.commandStatus(), CMD_STATUS_MODE_REFUSED);

    ctl.updateFaults(0, CTRL_HEALTH_OK);
    CHECK(sendFrame(ctl, 0, 0, MODE_SAFE, false));
    CHECK_EQ(ctl.activeMode(), MODE_SAFE);
}

static void non_critical_faults_are_reported_but_do_not_stop_the_rover() {
    // These are what the planned DEGRADED state is for: the rover can still
    // finish its task or get home. Until then they are reported, not acted on.
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

static void a_critical_fault_in_safe_does_not_latch_but_blocks_driving() {
    // SAFE is not moving, so nothing latches there. Asking to move while the
    // fault is present enters the mode and faults on the same frame, which
    // the operator sees as FAULT with the reason in the fault word.
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

int main() {
    std::printf("test_controller\n");
    RUN_TEST(boots_stopped_and_safe);
    RUN_TEST(valid_control_releases_the_stop);
    RUN_TEST(watchdog_trips_exactly_at_the_boundary);
    RUN_TEST(watchdog_is_correct_across_millis_wraparound);
    RUN_TEST(explicit_stop_flag_wins_in_any_mode);
    RUN_TEST(stop_is_releasable_without_a_mode_change);
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
    RUN_TEST(a_stale_source_mismatch_is_not_reported);
    RUN_TEST(an_unauthorised_frame_still_feeds_the_watchdog);
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
    return testing::summary("test_controller");
}
