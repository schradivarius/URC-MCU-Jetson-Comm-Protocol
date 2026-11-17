#include "../../firmware/src/rover_estop.h"
#include "test_framework.h"

using namespace rover;

static uint32_t hold(EstopInput& in, uint32_t t, uint32_t ms, bool raw) {
    for (uint32_t i = 0; i < ms; ++i) in.update(raw, ++t);
    return t;
}

static void a_fresh_input_reads_inactive() {
    EstopInput in;
    CHECK(!in.active());
    CHECK(!in.seeded());
    CHECK_EQ(in.assertions(), 0u);
}

static void the_first_reading_is_taken_without_debounce() {
    EstopInput in;
    in.update(true, 1000);
    CHECK(in.active());
    CHECK(in.seeded());
    CHECK_EQ(in.assertions(), 1u);
}

static void a_clear_first_reading_does_not_assert() {
    EstopInput in;
    in.update(false, 1000);
    CHECK(!in.active());
    CHECK(in.seeded());
    CHECK_EQ(in.assertions(), 0u);
}

static void a_press_asserts_only_after_the_debounce_window() {
    EstopInput in;
    uint32_t t = 100;
    in.update(false, t);
    t = hold(in, t, ESTOP_DEBOUNCE_MS, true);
    CHECK(!in.active());
    t = hold(in, t, 1, true);
    CHECK(in.active());
    CHECK_EQ(in.assertions(), 1u);
}

static void a_release_clears_only_after_the_debounce_window() {
    EstopInput in;
    uint32_t t = 100;
    in.update(false, t);
    t = hold(in, t, ESTOP_DEBOUNCE_MS + 1, true);
    CHECK(in.active());
    t = hold(in, t, ESTOP_DEBOUNCE_MS, false);
    CHECK(in.active());
    t = hold(in, t, 1, false);
    CHECK(!in.active());
}

static void bounce_never_promotes_a_press() {
    EstopInput in;
    uint32_t t = 100;
    in.update(false, t);
    for (uint32_t i = 0; i < 200; ++i) {
        in.update(i % 2 == 0, ++t);
        CHECK(!in.active());
    }
    CHECK_EQ(in.assertions(), 0u);
}

static void bounce_never_promotes_a_release() {
    EstopInput in;
    uint32_t t = 100;
    in.update(true, t);
    CHECK(in.active());
    for (uint32_t i = 0; i < 200; ++i) {
        in.update(i % 2 == 0, ++t);
        CHECK(in.active());
    }
}

static void the_window_is_measured_from_the_last_bounce() {
    EstopInput in;
    uint32_t t = 100;
    in.update(false, t);
    t = hold(in, t, ESTOP_DEBOUNCE_MS, true);
    in.update(false, ++t);
    t = hold(in, t, ESTOP_DEBOUNCE_MS, true);
    CHECK(!in.active());
    t = hold(in, t, 1, true);
    CHECK(in.active());
}

static void a_settled_signal_does_not_reassert() {
    EstopInput in;
    uint32_t t = 100;
    in.update(false, t);
    t = hold(in, t, ESTOP_DEBOUNCE_MS + 1, true);
    CHECK_EQ(in.assertions(), 1u);
    t = hold(in, t, 500, true);
    CHECK(in.active());
    CHECK_EQ(in.assertions(), 1u);
}

static void each_distinct_press_counts_once() {
    EstopInput in;
    uint32_t t = 100;
    in.update(false, t);
    for (int cycle = 0; cycle < 3; ++cycle) {
        t = hold(in, t, ESTOP_DEBOUNCE_MS + 1, true);
        CHECK(in.active());
        t = hold(in, t, ESTOP_DEBOUNCE_MS + 1, false);
        CHECK(!in.active());
    }
    CHECK_EQ(in.assertions(), 3u);
}

static void the_counter_survives_a_millis_rollover() {
    EstopInput in;
    uint32_t t = 0xFFFFFFFFu - 5u;
    in.update(false, t);
    for (uint32_t i = 0; i <= ESTOP_DEBOUNCE_MS; ++i) in.update(true, ++t);
    CHECK(in.active());
    CHECK_EQ(in.assertions(), 1u);
}

int main() {
    std::printf("test_estop\n");
    RUN_TEST(a_fresh_input_reads_inactive);
    RUN_TEST(the_first_reading_is_taken_without_debounce);
    RUN_TEST(a_clear_first_reading_does_not_assert);
    RUN_TEST(a_press_asserts_only_after_the_debounce_window);
    RUN_TEST(a_release_clears_only_after_the_debounce_window);
    RUN_TEST(bounce_never_promotes_a_press);
    RUN_TEST(bounce_never_promotes_a_release);
    RUN_TEST(the_window_is_measured_from_the_last_bounce);
    RUN_TEST(a_settled_signal_does_not_reassert);
    RUN_TEST(each_distinct_press_counts_once);
    RUN_TEST(the_counter_survives_a_millis_rollover);
    return testing::summary("test_estop");
}
