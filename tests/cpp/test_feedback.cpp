#include "../../firmware/src/rover_feedback.h"
#include "test_framework.h"

using namespace rover;

static uint32_t g_now;
static uint32_t fakeMillis() { return g_now; }
static void advance(uint32_t ms) { g_now += ms; } 

static FeedbackAge freshTracker(uint32_t start_ms = 0) {
    g_now = start_ms;
    return FeedbackAge(fakeMillis, 300);
}

static void age_is_unknown_before_the_first_sample() {
    FeedbackAge age = freshTracker();
    CHECK(age.ageMs() == CMD_AGE_UNKNOWN);
}

static void stale_before_the_first_sample() {
    FeedbackAge age = freshTracker();
    CHECK(age.isStale());
}

static void age_grows_with_the_clock() {
    FeedbackAge age = freshTracker();
    age.noteSample();
    advance(100);
    CHECK_EQ(age.ageMs(), 100);
}

static void a_sample_resets_the_age() {
    FeedbackAge age = freshTracker();
    age.noteSample();
    advance(100);
    CHECK_EQ(age.ageMs(), 100);
    age.noteSample();
    CHECK_EQ(age.ageMs(), 0);
}

static void repeated_samples_keep_it_fresh() {
    FeedbackAge age = freshTracker();
    age.noteSample();
    for (int i = 0; i < 20; i++) {
        advance(50);
        CHECK_EQ(age.ageMs(), 50);
        age.noteSample();
        CHECK(!age.isStale());
    }    
}
static void a_new_sample_clears_the_stale_flag() {
    FeedbackAge age = freshTracker();
    age.noteSample();
    advance(400);
    CHECK(age.isStale());
    age.noteSample();
    CHECK(!age.isStale());
}

static void stale_at_the_threshold() {
    FeedbackAge age = freshTracker();
    age.noteSample();
    advance(300);
    CHECK(age.isStale());
}

static void stale_after_the_threshold() {
    FeedbackAge age = freshTracker();
    age.noteSample();
    advance(301);
    CHECK(age.isStale());
}

static void not_stale_1_ms_before_the_threshold() {
    FeedbackAge age = freshTracker();
    age.noteSample();
    advance(299);
    CHECK_EQ(age.ageMs(), 299);
    CHECK(!age.isStale());
}

static void age_is_correct_across_millis_wrap() {
    FeedbackAge age = freshTracker(0xFFFFFF00u);
    age.noteSample();
    advance(200);
    CHECK_EQ(age.ageMs(), 200);
    CHECK(!age.isStale());
    advance(100);
    CHECK_EQ(age.ageMs(), 300);
    CHECK(age.isStale());
}

static void age_saturates_at_max_instead_of_wrapping() {
    FeedbackAge age = freshTracker();
    age.noteSample();

    advance(65534);                          // the largest age that fits
    CHECK_EQ(age.ageMs(), 65534);            // sent as-is

    advance(1);                              // 65,535 ms: one past
    CHECK_EQ(age.ageMs(), CMD_AGE_MAX);      // pinned at the top...
    CHECK(age.ageMs() != CMD_AGE_UNKNOWN);   // ...not mistaken for "never"

    advance(70000 - 65535);                  // 70 seconds since the sample
    CHECK_EQ(age.ageMs(), CMD_AGE_MAX);      // still pinned, not 4,464
    CHECK(age.isStale());
}

int main() {
    RUN_TEST(age_is_unknown_before_the_first_sample);
    RUN_TEST(stale_before_the_first_sample);
    RUN_TEST(age_grows_with_the_clock);
    RUN_TEST(a_sample_resets_the_age);
    RUN_TEST(repeated_samples_keep_it_fresh);
    RUN_TEST(a_new_sample_clears_the_stale_flag);
    RUN_TEST(stale_at_the_threshold);
    RUN_TEST(stale_after_the_threshold);
    RUN_TEST(not_stale_1_ms_before_the_threshold);
    RUN_TEST(age_is_correct_across_millis_wrap);
    RUN_TEST(age_saturates_at_max_instead_of_wrapping);


    return testing::summary("test_feedback");
}