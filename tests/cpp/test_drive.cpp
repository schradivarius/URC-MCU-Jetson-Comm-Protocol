#include "../../firmware/src/rover_drive.h"
#include "test_framework.h"

using namespace rover;

static void boots_at_neutral_and_reports_nothing_known() {
    DriveModule drv;
    CHECK(!drv.outputs().energized);
    CHECK_EQ(drv.outputs().left_effort, 0);
    CHECK_EQ(drv.outputs().right_effort, 0);
    CHECK_EQ(drv.outputs().steer_effort, 0);
    CHECK_EQ(drv.outputs().left_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(drv.outputs().right_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(drv.outputs().steer_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(drv.health(), CTRL_HEALTH_NOT_REPORTED);
}

static void a_de_energized_update_puts_every_channel_at_neutral() {
    DriveModule drv;
    const DriveOutputs& o = drv.update(1000, 1000, false);
    CHECK(!o.energized);
    CHECK_EQ(o.left_effort, 0);
    CHECK_EQ(o.right_effort, 0);
    CHECK_EQ(o.steer_effort, 0);
    CHECK_EQ(o.left_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(o.right_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(o.steer_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(drv.health(), CTRL_HEALTH_OK);
}

static void de_energizing_takes_effect_in_the_same_cycle() {
    DriveModule drv;
    drv.update(1000, 0, true);
    CHECK_EQ(drv.outputs().left_effort, 1000);
    drv.update(1000, 0, false);
    CHECK_EQ(drv.outputs().left_effort, 0);
    CHECK_EQ(drv.outputs().left_pwm_us, PWM_NEUTRAL_US);
}

static void straight_ahead_drives_both_sides_equally() {
    DriveModule drv;
    const DriveOutputs& o = drv.update(600, 0, true);
    CHECK_EQ(o.left_effort, 600);
    CHECK_EQ(o.right_effort, 600);
    CHECK_EQ(o.steer_effort, 0);
    CHECK(o.energized);
    CHECK_EQ(drv.health(), CTRL_HEALTH_OK);
}

static void a_turn_in_place_drives_the_sides_opposite() {
    DriveModule drv;
    const DriveOutputs& o = drv.update(0, 400, true);
    CHECK_EQ(o.left_effort, 400);
    CHECK_EQ(o.right_effort, -400);
    CHECK_EQ(o.steer_effort, 400);
}

static void a_saturating_mix_is_scaled_not_clamped() {
    DriveModule drv;
    const DriveOutputs& o = drv.update(800, 400, true);
    CHECK_EQ(o.left_effort, 1000);
    CHECK_EQ(o.right_effort, 333);
    CHECK_EQ(drv.health(), CTRL_HEALTH_DEGRADED);
}

static void saturation_is_live_not_latched() {
    DriveModule drv;
    drv.update(800, 400, true);
    CHECK_EQ(drv.health(), CTRL_HEALTH_DEGRADED);
    drv.update(400, 200, true);
    CHECK_EQ(drv.health(), CTRL_HEALTH_OK);
}

static void a_scaled_mix_never_exceeds_full_scale() {
    DriveModule drv;
    for (int d = CMD_MIN; d <= CMD_MAX; d += 25) {
        for (int s = CMD_MIN; s <= CMD_MAX; s += 25) {
            const DriveOutputs& o = drv.update(static_cast<int16_t>(d),
                                               static_cast<int16_t>(s), true);
            CHECK(o.left_effort >= CMD_MIN && o.left_effort <= CMD_MAX);
            CHECK(o.right_effort >= CMD_MIN && o.right_effort <= CMD_MAX);
            CHECK(o.left_pwm_us >= PWM_NEUTRAL_US - PWM_SPAN_US);
            CHECK(o.left_pwm_us <= PWM_NEUTRAL_US + PWM_SPAN_US);
            CHECK(o.right_pwm_us >= PWM_NEUTRAL_US - PWM_SPAN_US);
            CHECK(o.right_pwm_us <= PWM_NEUTRAL_US + PWM_SPAN_US);
        }
    }
    CHECK_EQ(drv.rangeViolations(), 0u);
}

static void the_turn_direction_is_never_inverted_by_scaling() {
    DriveModule drv;
    for (int d = CMD_MIN; d <= CMD_MAX; d += 50) {
        for (int s = 50; s <= CMD_MAX; s += 50) {
            const DriveOutputs& o = drv.update(static_cast<int16_t>(d),
                                               static_cast<int16_t>(s), true);
            CHECK(o.left_effort >= o.right_effort);
        }
    }
}

static void pwm_maps_the_command_range_onto_the_pulse_range() {
    CHECK_EQ(effortToPwmUs(0), PWM_NEUTRAL_US);
    CHECK_EQ(effortToPwmUs(CMD_MAX), PWM_NEUTRAL_US + PWM_SPAN_US);
    CHECK_EQ(effortToPwmUs(CMD_MIN), PWM_NEUTRAL_US - PWM_SPAN_US);
    CHECK_EQ(effortToPwmUs(500), PWM_NEUTRAL_US + PWM_SPAN_US / 2);
    CHECK_EQ(effortToPwmUs(-500), PWM_NEUTRAL_US - PWM_SPAN_US / 2);
}

static void pwm_clamps_rather_than_wrapping_past_full_scale() {
    CHECK_EQ(effortToPwmUs(32767), PWM_NEUTRAL_US + PWM_SPAN_US);
    CHECK_EQ(effortToPwmUs(-32768), PWM_NEUTRAL_US - PWM_SPAN_US);
}

static void pwm_is_monotonic_across_the_whole_command_range() {
    uint16_t previous = effortToPwmUs(CMD_MIN);
    for (int v = CMD_MIN + 1; v <= CMD_MAX; ++v) {
        const uint16_t now = effortToPwmUs(static_cast<int16_t>(v));
        CHECK(now >= previous);
        previous = now;
    }
    CHECK_EQ(previous, PWM_NEUTRAL_US + PWM_SPAN_US);
}

static void an_out_of_range_command_is_clamped_and_latched_as_a_fault() {
    DriveModule drv;
    const DriveOutputs& o = drv.update(5000, 0, true);
    CHECK_EQ(o.left_effort, CMD_MAX);
    CHECK_EQ(o.right_effort, CMD_MAX);
    CHECK_EQ(drv.rangeViolations(), 1u);
    CHECK_EQ(drv.health(), CTRL_HEALTH_FAULT);

    drv.update(100, 0, true);
    CHECK_EQ(drv.health(), CTRL_HEALTH_FAULT);
    CHECK_EQ(drv.rangeViolations(), 1u);
}

static void an_out_of_range_steer_is_caught_too() {
    DriveModule drv;
    drv.update(0, -5000, true);
    CHECK_EQ(drv.outputs().steer_effort, CMD_MIN);
    CHECK_EQ(drv.health(), CTRL_HEALTH_FAULT);
}

static void an_out_of_range_command_while_stopped_is_still_caught() {
    DriveModule drv;
    drv.update(9000, 0, false);
    CHECK_EQ(drv.outputs().left_effort, 0);
    CHECK_EQ(drv.health(), CTRL_HEALTH_FAULT);
}

static void the_boundary_values_are_not_violations() {
    DriveModule drv;
    drv.update(CMD_MAX, 0, true);
    drv.update(CMD_MIN, 0, true);
    drv.update(0, CMD_MAX, true);
    drv.update(0, CMD_MIN, true);
    CHECK_EQ(drv.rangeViolations(), 0u);
    CHECK(drv.health() != CTRL_HEALTH_FAULT);
}

static void update_counting_survives_a_long_run() {
    DriveModule drv;
    for (int i = 0; i < 1000; ++i) drv.update(100, 50, true);
    CHECK_EQ(drv.updates(), 1000u);
}

int main() {
    RUN_TEST(boots_at_neutral_and_reports_nothing_known);
    RUN_TEST(a_de_energized_update_puts_every_channel_at_neutral);
    RUN_TEST(de_energizing_takes_effect_in_the_same_cycle);
    RUN_TEST(straight_ahead_drives_both_sides_equally);
    RUN_TEST(a_turn_in_place_drives_the_sides_opposite);
    RUN_TEST(a_saturating_mix_is_scaled_not_clamped);
    RUN_TEST(saturation_is_live_not_latched);
    RUN_TEST(a_scaled_mix_never_exceeds_full_scale);
    RUN_TEST(the_turn_direction_is_never_inverted_by_scaling);
    RUN_TEST(pwm_maps_the_command_range_onto_the_pulse_range);
    RUN_TEST(pwm_clamps_rather_than_wrapping_past_full_scale);
    RUN_TEST(pwm_is_monotonic_across_the_whole_command_range);
    RUN_TEST(an_out_of_range_command_is_clamped_and_latched_as_a_fault);
    RUN_TEST(an_out_of_range_steer_is_caught_too);
    RUN_TEST(an_out_of_range_command_while_stopped_is_still_caught);
    RUN_TEST(the_boundary_values_are_not_violations);
    RUN_TEST(update_counting_survives_a_long_run);
    return testing::summary("test_drive");
}
