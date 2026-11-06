// test_steer_sim.cpp -- simulated steering feedback.

#include "../../firmware/src/rover_steer_sim.h"
#include "test_framework.h"

using namespace rover;

static const int16_t TURN = 300;

// Runs `ms` updates 1 ms apart starting after `t`; returns the new time.
static uint32_t run(SteerSim& sim, uint32_t t, uint32_t ms, int16_t target) {
    for (uint32_t i = 0; i < ms; ++i) sim.update(target, ++t);
    return t;
}

static void nothing_is_valid_before_the_first_update() {
    SteerSim sim;
    const SteerFeedback f = sim.feedback(0);
    CHECK(!f.valid);
    CHECK_EQ(f.measured, static_cast<int16_t>(0));
    CHECK_EQ(f.commanded, static_cast<int16_t>(0));
    CHECK_EQ(f.error, static_cast<int16_t>(0));
}

// The whole point of the module: the actuator must not teleport.
static void measured_does_not_jump_to_the_command() {
    SteerSim sim;
    uint32_t t = run(sim, 0, 1, 0);
    t = run(sim, t, 10, TURN);
    CHECK_EQ(sim.feedback(t).commanded, TURN);
    CHECK(sim.measured() > 0);
    CHECK(sim.measured() < TURN);
}

static void measured_moves_through_the_range_and_arrives() {
    SteerSim sim;
    uint32_t t = run(sim, 0, 1, 0);
    int16_t prev = 0;
    int distinct = 0;
    for (int step = 0; step < 6; ++step) {
        t = run(sim, t, 20, TURN);
        CHECK(sim.measured() >= prev);
        if (sim.measured() != prev) distinct++;
        prev = sim.measured();
    }
    // The task asks for feedback that visibly passes through intermediate
    // values rather than snapping, so several distinct readings is the
    // requirement, not an implementation detail.
    CHECK(distinct >= 3);
    t = run(sim, t, 500, TURN);
    CHECK_EQ(sim.measured(), TURN);
}

static void it_never_overshoots_the_target() {
    SteerSim sim;
    uint32_t t = run(sim, 0, 1, 0);
    for (uint32_t i = 0; i < 2000; ++i) {
        t = run(sim, t, 1, TURN);
        CHECK(sim.measured() <= TURN);
    }
    CHECK_EQ(sim.measured(), TURN);
}

static void it_steers_the_other_way_too() {
    SteerSim sim;
    uint32_t t = run(sim, 0, 1, 0);
    t = run(sim, t, 500, CMD_MIN);
    CHECK_EQ(sim.measured(), static_cast<int16_t>(CMD_MIN));
    for (uint32_t i = 0; i < 2000; ++i) {
        t = run(sim, t, 1, CMD_MIN);
        CHECK(sim.measured() >= CMD_MIN);
    }
}

static void it_reverses_when_the_target_crosses_the_centre() {
    SteerSim sim;
    uint32_t t = run(sim, 0, 1, 0);
    t = run(sim, t, 500, TURN);
    CHECK_EQ(sim.measured(), TURN);
    t = run(sim, t, 10, -TURN);
    CHECK(sim.measured() < TURN);
    t = run(sim, t, 500, -TURN);
    CHECK_EQ(sim.measured(), static_cast<int16_t>(-TURN));
}

// Task 5's "steering error" minimum-track item.
static void error_is_the_gap_and_closes_to_zero() {
    SteerSim sim;
    uint32_t t = run(sim, 0, 1, 0);
    t = run(sim, t, 10, TURN);
    const SteerFeedback mid = sim.feedback(t);
    CHECK_EQ(mid.error, static_cast<int16_t>(mid.commanded - mid.measured));
    CHECK(mid.error > 0);
    t = run(sim, t, 500, TURN);
    CHECK_EQ(sim.feedback(t).error, static_cast<int16_t>(0));
}

static void error_is_signed_the_way_a_controller_expects() {
    SteerSim sim;
    uint32_t t = run(sim, 0, 1, 0);
    t = run(sim, t, 500, TURN);
    t = run(sim, t, 10, -TURN);
    CHECK(sim.feedback(t).error < 0);
}

// Task 5's "feedback update time" minimum-track item.
static void the_update_time_is_recorded() {
    SteerSim sim;
    sim.update(0, 1234);
    CHECK_EQ(sim.lastUpdateMs(), 1234u);
    CHECK_EQ(sim.feedback(1300).last_update_ms, 1234u);
}

static void feedback_goes_invalid_once_it_stops_being_refreshed() {
    SteerSim sim;
    sim.update(0, 1000);
    CHECK(sim.feedback(1000).valid);
    CHECK(sim.feedback(1000 + STEER_STALE_MS).valid);
    CHECK(!sim.feedback(1000 + STEER_STALE_MS + 1).valid);
    sim.update(0, 1000 + STEER_STALE_MS + 1);
    CHECK(sim.feedback(1000 + STEER_STALE_MS + 1).valid);
}

// millis() wraps every ~49.7 days; unsigned subtraction has to carry it.
static void timing_is_correct_across_millis_wraparound() {
    SteerSim sim;
    const uint32_t before = 0xFFFFFFF0u;
    sim.update(0, before);
    CHECK(sim.feedback(before + 10).valid);
    CHECK(!sim.feedback(before + STEER_STALE_MS + 1).valid);

    SteerSim moving;
    moving.update(TURN, before);
    moving.update(TURN, before + 50);   // wraps past 0 inside the window
    CHECK(moving.measured() > 0);
    CHECK_EQ(moving.lastUpdateMs(), static_cast<uint32_t>(before + 50));
}

// A stalled loop did not steer for the length of the stall.
static void a_long_gap_is_clamped_to_one_step() {
    SteerSim sim;
    sim.update(CMD_MAX, 0);
    sim.update(CMD_MAX, 1);
    const int16_t after_1ms = sim.measured();
    SteerSim stalled;
    stalled.update(CMD_MAX, 0);
    stalled.update(CMD_MAX, 0xFFFFFFF0u);
    CHECK(stalled.measured() >= after_1ms);
    CHECK(stalled.measured() <= CMD_MAX);
}

static void a_zero_length_step_moves_nothing() {
    SteerSim sim;
    sim.update(TURN, 500);
    const int16_t first = sim.measured();
    sim.update(TURN, 500);
    CHECK_EQ(sim.measured(), first);
}

// With the default rate one millisecond is two whole units, but the module
// must not depend on that: a sub-unit step has to accumulate, not vanish.
static void a_sub_unit_step_accumulates_instead_of_vanishing() {
    SteerSim sim;
    uint32_t t = 0;
    sim.update(CMD_MAX, t);
    for (int i = 0; i < 100; ++i) sim.update(CMD_MAX, ++t);
    CHECK_EQ(sim.measured(),
             static_cast<int16_t>(STEER_SIM_RATE_PER_S * 100 / 1000));
}

int main() {
    std::printf("test_steer_sim\n");
    RUN_TEST(nothing_is_valid_before_the_first_update);
    RUN_TEST(measured_does_not_jump_to_the_command);
    RUN_TEST(measured_moves_through_the_range_and_arrives);
    RUN_TEST(it_never_overshoots_the_target);
    RUN_TEST(it_steers_the_other_way_too);
    RUN_TEST(it_reverses_when_the_target_crosses_the_centre);
    RUN_TEST(error_is_the_gap_and_closes_to_zero);
    RUN_TEST(error_is_signed_the_way_a_controller_expects);
    RUN_TEST(the_update_time_is_recorded);
    RUN_TEST(feedback_goes_invalid_once_it_stops_being_refreshed);
    RUN_TEST(timing_is_correct_across_millis_wraparound);
    RUN_TEST(a_long_gap_is_clamped_to_one_step);
    RUN_TEST(a_zero_length_step_moves_nothing);
    RUN_TEST(a_sub_unit_step_accumulates_instead_of_vanishing);
    return testing::summary("test_steer_sim");
}
