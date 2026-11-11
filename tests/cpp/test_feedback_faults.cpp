// test_feedback_faults.cpp -- the four controlled feedback failures, and the
// controller's reaction to each.
//
// These tests go all the way through: a healthy reading goes into the
// injector, the corrupted reading comes out, and the CONTROLLER is asked what
// it makes of it. Checking only the injector would prove a fake sensor can
// lie; the point is that the detector added in v0.8 catches the lie.

#include "../../firmware/src/rover_controller.h"
#include "../../firmware/src/rover_encoder.h"
#include "../../firmware/src/rover_feedback_faults.h"
#include "../../firmware/src/rover_steer_sim.h"
#include "test_framework.h"

using namespace rover;

static uint32_t g_now = 0;
static uint32_t fakeMillis() { return g_now; }

// One loop of a healthy rover: plant, injector, controller. `ms` advances the
// clock before each iteration, so the whole chain sees the same timeline the
// firmware would.
struct Rig {
    RoverController ctl;
    SimulatedEncoder enc;
    SteerSim steer;
    FeedbackFaultInjector faults;

    Rig() : ctl(fakeMillis, 300, 50), seq_(0), commanding_(false),
            drive_cmd_(0), steer_cmd_(0) { g_now = 1000; }

    // Start feeding real CONTROL frames so the rover is actually driving.
    // Without this the watchdog is tripped and the mode is SAFE, which stops
    // the rover for reasons that have nothing to do with feedback.
    void command(int16_t drive, int16_t steer) {
        commanding_ = true;
        drive_cmd_  = drive;
        steer_cmd_  = steer;
        send(MODE_TELEOP, 0, 0);          // the operator grants TELEOP first
        send(MODE_TELEOP, drive, steer);
    }

    void send(uint8_t mode, int16_t drive, int16_t steer) {
        ControlMsg m;
        m.drive_cmd = drive; m.steer_cmd = steer; m.mode = mode;
        m.stop = false; m.autonomy_abort = false; m.return_request = false;
        m.c2_lost = false; m.from_autonomy = false;
        m.indicator_request = INDICATOR_OFF; m.seq = ++seq_;
        uint8_t buf[8] = {0};
        ctl.ingestFrame(CAN_ID_CONTROL, buf, encodeControl(m, buf));
    }

    void step(uint32_t ms, int16_t drive_scaled = 300, int16_t steer_target = 0) {
        g_now += ms;
        if (commanding_) send(MODE_TELEOP, drive_cmd_, steer_cmd_);
        enc.update(drive_scaled, g_now);
        steer.update(steer_target, g_now);
        EncoderFeedback e = enc.feedback(g_now);
        SteerFeedback   s = steer.feedback(g_now);
        faults.apply(e, s, g_now);
        last_encoder = e;
        last_steer   = s;
        ctl.updateFeedback(e.last_update_ms,
                           e.rpm >= -ENCODER_MAX_RPM && e.rpm <= ENCODER_MAX_RPM,
                           s.last_update_ms,
                           isValidCommand(s.measured));
    }

    void run(uint32_t ms, uint32_t step_ms = 10, int16_t drive = 300,
             int16_t steer_target = 0) {
        for (uint32_t t = 0; t < ms; t += step_ms) step(step_ms, drive, steer_target);
    }

    EncoderFeedback last_encoder;
    SteerFeedback   last_steer;

private:
    uint8_t seq_;
    bool    commanding_;
    int16_t drive_cmd_;
    int16_t steer_cmd_;
};

// The control for every test below: with nothing armed, nothing is flagged.
// Without this, a detector that flagged everything would pass them all.
static void a_healthy_plant_raises_nothing() {
    Rig r;
    r.run(400);
    CHECK(!r.ctl.encoderStale());
    CHECK(!r.ctl.steerStale());
    CHECK(!r.ctl.encoderInvalid());
    CHECK(!r.ctl.steerInvalid());
    CHECK_EQ(r.ctl.buildFeedback().flags, static_cast<uint8_t>(0));
    CHECK((r.ctl.faultWord() & FAULT_STALE_FEEDBACK) == 0);
    CHECK((r.ctl.faultWord() & FAULT_ENCODER_FAULT) == 0);
    CHECK((r.ctl.faultWord() & FAULT_STEERING_FAULT) == 0);
}

// --- 1. encoder feedback stops updating ------------------------------------

static void a_frozen_encoder_is_detected_and_reported() {
    Rig r;
    r.run(200);
    r.faults.arm(FB_FAULT_ENCODER_FROZEN);
    r.step(10);                                   // the freeze takes its sample
    const int32_t count_when_frozen = r.last_encoder.count;
    r.run(400);

    // The reading itself is unremarkable -- same count, a plausible RPM. Only
    // the age gives it away, which is exactly the case Task 5 describes.
    CHECK_EQ(r.last_encoder.count, count_when_frozen);
    CHECK(r.ctl.encoderStale());
    CHECK(!r.ctl.encoderInvalid());
    CHECK(r.ctl.encoderAgeMs() >= FEEDBACK_STALE_MS);
    CHECK((r.ctl.buildFeedback().flags & FB_FLAG_ENCODER_STALE) != 0);
    CHECK((r.ctl.faultWord() & FAULT_STALE_FEEDBACK) != 0);
    // The other channel is untouched: a fault on one must not smear.
    CHECK(!r.ctl.steerStale());
    CHECK((r.ctl.buildFeedback().flags & FB_FLAG_STEER_STALE) == 0);
}

static void a_frozen_encoder_recovers_when_the_fault_is_cleared() {
    Rig r;
    r.run(100);
    r.faults.arm(FB_FAULT_ENCODER_FROZEN);
    r.run(400);
    CHECK(r.ctl.encoderStale());
    r.faults.arm(FB_FAULT_NONE);
    r.run(50);
    CHECK(!r.ctl.encoderStale());
    CHECK((r.ctl.faultWord() & FAULT_STALE_FEEDBACK) == 0);
}

// --- 2. steering feedback stops updating -----------------------------------

static void frozen_steering_is_detected_and_reported() {
    Rig r;
    r.run(200, 10, 300, 400);
    r.faults.arm(FB_FAULT_STEER_FROZEN);
    r.step(10, 300, 400);                         // the freeze takes its sample
    const int16_t measured_when_frozen = r.last_steer.measured;
    r.run(400, 10, 300, 400);

    CHECK_EQ(r.last_steer.measured, measured_when_frozen);
    CHECK(r.ctl.steerStale());
    CHECK(!r.ctl.steerInvalid());
    CHECK((r.ctl.buildFeedback().flags & FB_FLAG_STEER_STALE) != 0);
    CHECK((r.ctl.faultWord() & FAULT_STALE_FEEDBACK) != 0);
    CHECK(!r.ctl.encoderStale());
}

// --- 3. an impossible / invalid value --------------------------------------

static void an_impossible_encoder_value_is_detected_and_reported() {
    Rig r;
    r.run(200);
    r.faults.arm(FB_FAULT_ENCODER_IMPOSSIBLE);
    r.run(50);

    CHECK(r.last_encoder.rpm > ENCODER_MAX_RPM);
    CHECK(r.ctl.encoderInvalid());
    // Fresh and wrong. If this ever reads stale too, the two conditions have
    // been collapsed and the operator loses the distinction.
    CHECK(!r.ctl.encoderStale());
    CHECK((r.ctl.buildFeedback().flags & FB_FLAG_ENCODER_INVALID) != 0);
    CHECK((r.ctl.buildFeedback().flags & FB_FLAG_ENCODER_STALE) == 0);
    CHECK((r.ctl.faultWord() & FAULT_ENCODER_FAULT) != 0);
    CHECK((r.ctl.faultWord() & FAULT_STALE_FEEDBACK) == 0);
}

static void an_impossible_steering_value_is_detected_and_reported() {
    Rig r;
    r.run(200, 10, 300, 200);
    r.faults.arm(FB_FAULT_STEER_IMPOSSIBLE);
    r.run(50, 10, 300, 200);

    CHECK(!isValidCommand(r.last_steer.measured));
    CHECK(r.ctl.steerInvalid());
    CHECK(!r.ctl.steerStale());
    CHECK((r.ctl.buildFeedback().flags & FB_FLAG_STEER_INVALID) != 0);
    CHECK((r.ctl.faultWord() & FAULT_STEERING_FAULT) != 0);
    CHECK(!r.ctl.encoderInvalid());
}

// --- 4. feedback is intentionally delayed ----------------------------------

static void delayed_feedback_is_detected_and_reported() {
    Rig r;
    r.run(400);
    r.faults.arm(FB_FAULT_DELAYED);
    r.run(600);

    CHECK(r.ctl.encoderStale());
    CHECK(r.ctl.steerStale());
    // Delayed, not broken: the values are real readings, just old ones.
    CHECK(!r.ctl.encoderInvalid());
    CHECK(!r.ctl.steerInvalid());
    CHECK((r.ctl.faultWord() & FAULT_STALE_FEEDBACK) != 0);
    CHECK((r.ctl.faultWord() & FAULT_ENCODER_FAULT) == 0);
}

// Delayed and frozen must not look the same. A frozen channel's age grows
// without limit; a delayed one settles at the delay and stays there.
static void a_delayed_channel_is_not_a_frozen_one() {
    Rig r;
    r.run(400);
    r.faults.arm(FB_FAULT_DELAYED);
    r.run(600);
    const uint16_t age_early = r.ctl.encoderAgeMs();
    r.run(600);
    const uint16_t age_later = r.ctl.encoderAgeMs();

    CHECK(age_early >= FB_FAULT_DELAY_MS);
    CHECK(age_later < FB_FAULT_DELAY_MS * 2);

    Rig frozen;
    frozen.run(400);
    frozen.faults.arm(FB_FAULT_ENCODER_FROZEN);
    frozen.run(600);
    const uint16_t frozen_early = frozen.ctl.encoderAgeMs();
    frozen.run(600);
    CHECK(frozen.ctl.encoderAgeMs() > frozen_early);
}

// The delay the caller asked for, not the delay its loop rate allows. A ring
// sized in CALLS spans less time the faster apply() is called, and a fast
// caller then gets a shorter delay reported as if it were the real one --
// which is what the host simulator hit: a 64-call ring at its loop rate
// spanned 128 ms and reported a 128 ms "250 ms" delay.
static void the_delay_is_the_same_however_fast_apply_is_called() {
    for (uint32_t step_ms = 1; step_ms <= 10; ++step_ms) {
        Rig r;
        r.run(400, step_ms);
        r.faults.arm(FB_FAULT_DELAYED);
        r.run(1200, step_ms);
        const uint16_t age = r.ctl.encoderAgeMs();
        CHECK(age >= FB_FAULT_DELAY_MS);
        // Not merely "old": within one sample of the delay asked for, so a
        // delay line that drifted longer would fail here too.
        CHECK(age <= FB_FAULT_DELAY_MS + FB_FAULT_SAMPLE_MS + step_ms);
    }
}

static void a_delayed_channel_still_tracks_the_command() {
    Rig r;
    r.run(400, 10, 0, 0);
    r.faults.arm(FB_FAULT_DELAYED);
    const int32_t before = r.last_encoder.count;
    r.run(800, 10, 600, 0);
    // Late, but moving: the counts are rising because the rover really is
    // driving. A frozen channel would be flat here.
    CHECK(r.last_encoder.count > before);
}

// --- the rule Task 5 is explicit about -------------------------------------

// "For now, only detect and report the stale condition." The safe state is
// Task 6. Every one of the four failures must leave the rover driving.
static void no_injected_failure_stops_the_rover() {
    const uint8_t faults[] = {FB_FAULT_ENCODER_FROZEN, FB_FAULT_STEER_FROZEN,
                              FB_FAULT_ENCODER_IMPOSSIBLE,
                              FB_FAULT_STEER_IMPOSSIBLE, FB_FAULT_DELAYED};
    for (uint8_t f : faults) {
        Rig r;
        r.command(400, 200);
        r.run(200, 10, 400, 200);
        CHECK(!r.ctl.effectiveStop());             // driving before the fault
        r.faults.arm(f);
        r.run(600, 10, 400, 200);
        CHECK(!r.ctl.effectiveStop());
        CHECK_EQ(r.ctl.activeMode(), MODE_TELEOP);
        int16_t drive = 0, steer = 0;
        r.ctl.commandedOutputs(drive, steer);
        CHECK_EQ(drive, static_cast<int16_t>(400));
    }
}

// --- the name table --------------------------------------------------------

static void every_fault_has_a_name_that_parses_back() {
    for (uint8_t f = FB_FAULT_NONE; f <= FB_FAULT_MAX; ++f) {
        const char* name = feedbackFaultName(f);
        CHECK(name[0] != 'u');            // not "unknown"
        uint8_t parsed = 0xFF;
        CHECK(parseFeedbackFault(name, parsed));
        CHECK_EQ(parsed, f);
    }
}

// A typo must not quietly run a healthy plant and let the test pass.
static void an_unknown_name_is_refused_rather_than_ignored() {
    uint8_t parsed = FB_FAULT_DELAYED;
    CHECK(!parseFeedbackFault("encoder-frozn", parsed));
    CHECK_EQ(parsed, static_cast<uint8_t>(FB_FAULT_DELAYED));  // left alone
    CHECK(!parseFeedbackFault("", parsed));
    CHECK(!parseFeedbackFault(nullptr, parsed));
    // A prefix of a real name is not that name.
    CHECK(!parseFeedbackFault("delay", parsed));
    CHECK(!parseFeedbackFault("delayedd", parsed));
}

static void arming_out_of_range_falls_back_to_none() {
    FeedbackFaultInjector inj;
    inj.arm(200);
    CHECK_EQ(inj.armed(), static_cast<uint8_t>(FB_FAULT_NONE));
}

// Re-arming the same fault every loop must not restart it, or a caller that
// sets it unconditionally would freeze a fresh reading each cycle and the
// channel would never go stale.
static void re_arming_the_same_fault_does_not_restart_it() {
    Rig r;
    r.run(100);
    for (int i = 0; i < 40; ++i) {
        r.faults.arm(FB_FAULT_ENCODER_FROZEN);
        r.run(10);
    }
    CHECK(r.ctl.encoderStale());
}

static void timing_is_correct_across_millis_wraparound() {
    Rig r;
    g_now = 0xFFFFFF00u;
    r.run(200);
    r.faults.arm(FB_FAULT_DELAYED);
    r.run(600);                       // wraps past zero mid-run
    CHECK(r.ctl.encoderStale());
    CHECK(!r.ctl.encoderInvalid());
}

int main() {
    std::printf("test_feedback_faults\n");
    RUN_TEST(a_healthy_plant_raises_nothing);
    RUN_TEST(a_frozen_encoder_is_detected_and_reported);
    RUN_TEST(a_frozen_encoder_recovers_when_the_fault_is_cleared);
    RUN_TEST(frozen_steering_is_detected_and_reported);
    RUN_TEST(an_impossible_encoder_value_is_detected_and_reported);
    RUN_TEST(an_impossible_steering_value_is_detected_and_reported);
    RUN_TEST(delayed_feedback_is_detected_and_reported);
    RUN_TEST(a_delayed_channel_is_not_a_frozen_one);
    RUN_TEST(the_delay_is_the_same_however_fast_apply_is_called);
    RUN_TEST(a_delayed_channel_still_tracks_the_command);
    RUN_TEST(no_injected_failure_stops_the_rover);
    RUN_TEST(every_fault_has_a_name_that_parses_back);
    RUN_TEST(an_unknown_name_is_refused_rather_than_ignored);
    RUN_TEST(arming_out_of_range_falls_back_to_none);
    RUN_TEST(re_arming_the_same_fault_does_not_restart_it);
    RUN_TEST(timing_is_correct_across_millis_wraparound);
    return testing::summary("test_feedback_faults");
}
