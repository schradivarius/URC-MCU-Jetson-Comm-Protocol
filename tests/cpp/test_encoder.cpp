#include "../../firmware/src/rover_encoder.h"

#include "test_framework.h"

using namespace rover;

static void run(SimulatedEncoder& enc, int16_t effort, uint32_t start, uint32_t ms) {
    for (uint32_t t = 0; t <= ms; ++t) enc.update(effort, start + t);
}

static void starts_at_zero_and_not_valid() {
    SimulatedEncoder enc;
    EncoderFeedback f = enc.feedback(0);
    CHECK_EQ(f.count, 0);
    CHECK_EQ(f.rpm, 0);
    CHECK(!f.valid); 
}

static void full_forward_for_one_second_turns_max_rpm_worth_of_ticks() {
    SimulatedEncoder enc;
    run(enc, CMD_MAX, 0, 1000);

    CHECK_EQ(enc.feedback(1000).count, ENCODER_MAX_RPM * ENCODER_TICKS_PER_REV / 60);
    CHECK_EQ(enc.feedback(1000).rpm, ENCODER_MAX_RPM);
}

static void reverse_counts_down() {
    SimulatedEncoder enc;
    run(enc, CMD_MIN, 0, 1000);
    CHECK_EQ(enc.feedback(1000).count, -(ENCODER_MAX_RPM * ENCODER_TICKS_PER_REV / 60));
    CHECK_EQ(enc.feedback(1000).rpm, -ENCODER_MAX_RPM);
}

static void zero_effort_holds_the_count() {
    SimulatedEncoder enc;
    run(enc, 500, 0, 500);
    const int32_t before = enc.feedback(500).count;
    run(enc, 0, 501, 500);
    CHECK_EQ(enc.feedback(1001).count, before);
    CHECK_EQ(enc.feedback(1001).rpm, 0);
}

static void a_slow_wheel_still_moves() {

    SimulatedEncoder enc;
    run(enc, 1, 0, 10000);
    CHECK(enc.feedback(10000).count > 0);
}

static void timestamp_is_the_time_of_the_last_update() {
    SimulatedEncoder enc;
    enc.update(300, 1234);
    CHECK_EQ(enc.feedback(1300).last_update_ms, 1234u);
}

static void feedback_goes_stale_when_updates_stop() {
    SimulatedEncoder enc;
    enc.update(300, 0);
    CHECK(enc.feedback(ENCODER_STALE_MS).valid);
    CHECK(!enc.feedback(ENCODER_STALE_MS + 1).valid);
    enc.update(300, ENCODER_STALE_MS + 1);  
    CHECK(enc.feedback(ENCODER_STALE_MS + 1).valid);
}

static void counting_survives_millis_wraparound() {
    SimulatedEncoder enc;
    const uint32_t start = 0xFFFFFFFFu - 499;   
    run(enc, CMD_MAX, start, 1000);
    const uint32_t end = start + 1000;  
    CHECK_EQ(enc.feedback(end).count, ENCODER_MAX_RPM * ENCODER_TICKS_PER_REV / 60);
    CHECK(enc.feedback(end).valid);
}

int main() {
    RUN_TEST(starts_at_zero_and_not_valid);
    RUN_TEST(full_forward_for_one_second_turns_max_rpm_worth_of_ticks);
    RUN_TEST(reverse_counts_down);
    RUN_TEST(zero_effort_holds_the_count);
    RUN_TEST(a_slow_wheel_still_moves);
    RUN_TEST(timestamp_is_the_time_of_the_last_update);
    RUN_TEST(feedback_goes_stale_when_updates_stop);
    RUN_TEST(counting_survives_millis_wraparound);
    return testing::summary("test_encoder");
}
