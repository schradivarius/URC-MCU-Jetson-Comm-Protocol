// test_drive_sim.cpp -- simulated drive feedback.

#include "../../firmware/src/rover_drive_sim.h"
#include "test_framework.h"

using namespace rover;

static const int16_t HALF = 500;   // 0.50 m/s with SIM_MAX_SPEED_MPS = 1.0

// Runs `ms` of 1 ms updates starting at `t`, returns the new time.
static uint32_t run(DriveSim& sim, uint32_t t, uint32_t ms, int16_t l, int16_t r,
                    bool energized = true) {
    for (uint32_t i = 0; i < ms; ++i) sim.update(l, r, energized, ++t);
    return t;
}

static bool near(float a, float b, float tol) {
    return (a > b ? a - b : b - a) <= tol;
}

static void measured_does_not_jump_to_the_command() {
    DriveSim sim;
    uint32_t t = run(sim, 0, 1, 0, 0);
    t = run(sim, t, 50, HALF, HALF);
    CHECK(near(sim.left().commanded_mps, 0.50f, 0.001f));
    CHECK(sim.left().measured_mps > 0.0f);
    CHECK(sim.left().measured_mps < 0.45f);
}

static void measured_rises_over_time_and_reaches_the_command() {
    DriveSim sim;
    uint32_t t = run(sim, 0, 1, 0, 0);
    float prev = 0.0f;
    for (int step = 0; step < 6; ++step) {
        t = run(sim, t, 50, HALF, HALF);
        CHECK(sim.left().measured_mps >= prev);
        prev = sim.left().measured_mps;
    }
    t = run(sim, t, 700, HALF, HALF);
    CHECK(near(sim.left().measured_mps, 0.50f, 0.01f));
    CHECK(sim.left().measured_mps <= 0.50f);
}

static void reverse_ramps_negative() {
    DriveSim sim;
    uint32_t t = run(sim, 0, 1, 0, 0);
    run(sim, t, 1000, -HALF, -HALF);
    CHECK(near(sim.left().measured_mps, -0.50f, 0.01f));
    CHECK(sim.left().encoder_ticks < 0);
}

static void encoders_follow_measured_not_commanded() {
    DriveSim sim;
    uint32_t t = run(sim, 0, 1, 0, 0);
    run(sim, t, 1000, HALF, HALF);
    // 1 s at 0.50 m/s commanded would be 500 ticks; the ramp costs some.
    CHECK(sim.left().encoder_ticks > 300);
    CHECK(sim.left().encoder_ticks < 500);
    CHECK(near(sim.left().ticks_per_s, sim.left().measured_mps * SIM_TICKS_PER_M, 0.01f));
}

static void stop_brings_speed_to_exactly_zero_and_freezes_encoders() {
    DriveSim sim;
    uint32_t t = run(sim, 0, 1, 0, 0);
    t = run(sim, t, 1000, 1000, 1000);
    t = run(sim, t, 300, 1000, 1000, false);
    CHECK_EQ(sim.left().measured_mps, 0.0f);
    CHECK_EQ(sim.left().commanded_mps, 0.0f);
    const int32_t frozen = sim.left().encoder_ticks;
    run(sim, t, 500, 1000, 1000, false);
    CHECK_EQ(sim.left().encoder_ticks, frozen);
}

static void equal_commands_give_equal_wheels() {
    DriveSim sim(7);
    uint32_t t = run(sim, 0, 1, 0, 0);
    run(sim, t, 800, 400, 400);
    CHECK_EQ(sim.left().encoder_ticks, sim.right().encoder_ticks);
    CHECK_EQ(sim.left().measured_mps, sim.right().measured_mps);
}

static void same_seed_repeats_and_different_seed_differs() {
    DriveSim a(42), b(42), c(43);
    for (uint32_t t = 1; t <= 120; t += 10) {
        a.update(HALF, HALF, true, t);
        b.update(HALF, HALF, true, t);
        c.update(HALF, HALF, true, t);
    }
    CHECK_EQ(a.left().measured_mps, b.left().measured_mps);
    CHECK(a.left().measured_mps != c.left().measured_mps);
}

static void update_time_is_tracked_across_millis_wraparound() {
    DriveSim sim;
    sim.update(HALF, HALF, true, 0xFFFFFFF0u);
    sim.update(HALF, HALF, true, 0x00000010u);
    CHECK_EQ(sim.lastUpdateMs(), 0x00000010u);
    CHECK_EQ(sim.lastDtMs(), 0x20u);
    CHECK(sim.left().measured_mps > 0.0f);
}

int main() {
    std::printf("test_drive_sim\n");
    RUN_TEST(measured_does_not_jump_to_the_command);
    RUN_TEST(measured_rises_over_time_and_reaches_the_command);
    RUN_TEST(reverse_ramps_negative);
    RUN_TEST(encoders_follow_measured_not_commanded);
    RUN_TEST(stop_brings_speed_to_exactly_zero_and_freezes_encoders);
    RUN_TEST(equal_commands_give_equal_wheels);
    RUN_TEST(same_seed_repeats_and_different_seed_differs);
    RUN_TEST(update_time_is_tracked_across_millis_wraparound);
    return testing::summary("test_drive_sim");
}
