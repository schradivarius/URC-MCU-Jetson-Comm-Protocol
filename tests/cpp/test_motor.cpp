#include "../../firmware/src/rover_drive.h"
#include "../../firmware/src/rover_motor.h"
#include "test_framework.h"

using namespace rover;

static void full_scale_matches_the_resolution() {
    CHECK_EQ(pwmFullScale(12), (uint32_t)4095);
    CHECK_EQ(pwmFullScale(8), (uint32_t)255);
    CHECK_EQ(pwmFullScale(0), (uint32_t)0);
    CHECK_EQ(pwmFullScale(200), pwmFullScale(16));
}

static void neutral_servo_pulse_is_a_thirteenth_of_a_fifty_hertz_frame() {
    const uint32_t duty = servoPulseDuty(PWM_NEUTRAL_US, PWM_RESOLUTION_BITS,
                                         SERVO_PWM_FREQUENCY_HZ);
    CHECK_EQ(duty, (uint32_t)((1500u * 4095u) / 20000u));
}

static void servo_duty_is_monotonic_across_the_pulse_range() {
    uint32_t previous = 0;
    for (uint16_t pulse = PWM_NEUTRAL_US - PWM_SPAN_US;
         pulse <= PWM_NEUTRAL_US + PWM_SPAN_US; ++pulse) {
        const uint32_t duty = servoPulseDuty(pulse, PWM_RESOLUTION_BITS,
                                             SERVO_PWM_FREQUENCY_HZ);
        CHECK(duty >= previous);
        previous = duty;
    }
}

static void servo_duty_never_exceeds_full_scale() {
    CHECK(servoPulseDuty(60000, PWM_RESOLUTION_BITS, SERVO_PWM_FREQUENCY_HZ)
          <= pwmFullScale(PWM_RESOLUTION_BITS));
    CHECK_EQ(servoPulseDuty(60000, PWM_RESOLUTION_BITS, SERVO_PWM_FREQUENCY_HZ),
             pwmFullScale(PWM_RESOLUTION_BITS));
}

static void a_zero_frequency_is_refused_rather_than_dividing_by_zero() {
    CHECK_EQ(servoPulseDuty(PWM_NEUTRAL_US, PWM_RESOLUTION_BITS, 0), (uint32_t)0);
    CHECK_EQ(servoPulseDuty(PWM_NEUTRAL_US, PWM_RESOLUTION_BITS, 2000000),
             (uint32_t)0);
}

static void a_stopped_command_produces_no_duty_at_all() {
    const MotorDuty d = hbridgeDuty(0, PWM_RESOLUTION_BITS);
    CHECK_EQ(d.duty, (uint32_t)0);
    CHECK(!d.reverse);
}

static void full_forward_is_full_duty_forward() {
    const MotorDuty d = hbridgeDuty(CMD_MAX, PWM_RESOLUTION_BITS);
    CHECK_EQ(d.duty, pwmFullScale(PWM_RESOLUTION_BITS));
    CHECK(!d.reverse);
}

static void full_reverse_is_full_duty_with_the_direction_pin_set() {
    const MotorDuty d = hbridgeDuty(CMD_MIN, PWM_RESOLUTION_BITS);
    CHECK_EQ(d.duty, pwmFullScale(PWM_RESOLUTION_BITS));
    CHECK(d.reverse);
}

static void the_magnitude_is_symmetric_about_zero() {
    for (int16_t effort = 1; effort <= CMD_MAX; ++effort) {
        const MotorDuty forward = hbridgeDuty(effort, PWM_RESOLUTION_BITS);
        const MotorDuty back = hbridgeDuty(static_cast<int16_t>(-effort),
                                           PWM_RESOLUTION_BITS);
        CHECK_EQ(forward.duty, back.duty);
        CHECK(!forward.reverse);
        CHECK(back.reverse);
    }
}

static void the_deadband_suppresses_a_buzz_but_nothing_above_it() {
    CHECK_EQ(hbridgeDuty(MOTOR_DEADBAND_EFFORT, PWM_RESOLUTION_BITS).duty,
             (uint32_t)0);
    CHECK_EQ(hbridgeDuty(static_cast<int16_t>(-MOTOR_DEADBAND_EFFORT),
                         PWM_RESOLUTION_BITS).duty, (uint32_t)0);
    CHECK(hbridgeDuty(static_cast<int16_t>(MOTOR_DEADBAND_EFFORT + 1),
                      PWM_RESOLUTION_BITS).duty > 0);
}

static void the_deadband_is_small_enough_to_still_crawl() {
    CHECK(MOTOR_DEADBAND_EFFORT > 0);
    CHECK(MOTOR_DEADBAND_EFFORT < CMD_MAX / 10);
}

static void duty_is_monotonic_and_bounded_across_the_command_range() {
    uint32_t previous = 0;
    for (int16_t effort = 0; effort <= CMD_MAX; ++effort) {
        const MotorDuty d = hbridgeDuty(effort, PWM_RESOLUTION_BITS);
        CHECK(d.duty >= previous);
        CHECK(d.duty <= pwmFullScale(PWM_RESOLUTION_BITS));
        previous = d.duty;
    }
}

static void an_out_of_range_effort_is_clamped_not_wrapped() {
    const MotorDuty high = hbridgeDuty(32767, PWM_RESOLUTION_BITS);
    CHECK_EQ(high.duty, pwmFullScale(PWM_RESOLUTION_BITS));
    CHECK(!high.reverse);

    const MotorDuty low = hbridgeDuty(-32768, PWM_RESOLUTION_BITS);
    CHECK_EQ(low.duty, pwmFullScale(PWM_RESOLUTION_BITS));
    CHECK(low.reverse);
}

static void a_de_energized_drive_stage_reaches_both_output_modes_as_a_stop() {
    DriveModule stage;
    const DriveOutputs& out = stage.update(CMD_MAX, CMD_MAX, false);

    CHECK_EQ(hbridgeDuty(out.left_effort, PWM_RESOLUTION_BITS).duty, (uint32_t)0);
    CHECK_EQ(hbridgeDuty(out.right_effort, PWM_RESOLUTION_BITS).duty, (uint32_t)0);
    CHECK_EQ(hbridgeDuty(out.steer_effort, PWM_RESOLUTION_BITS).duty, (uint32_t)0);

    CHECK_EQ(out.left_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(servoPulseDuty(out.left_pwm_us, PWM_RESOLUTION_BITS,
                            SERVO_PWM_FREQUENCY_HZ),
             servoPulseDuty(PWM_NEUTRAL_US, PWM_RESOLUTION_BITS,
                            SERVO_PWM_FREQUENCY_HZ));
}

static void a_commanded_drive_reaches_the_h_bridge_as_duty() {
    DriveModule stage;
    const DriveOutputs& out = stage.update(500, 0, true);
    CHECK_EQ(out.left_effort, (int16_t)500);
    const MotorDuty d = hbridgeDuty(out.left_effort, PWM_RESOLUTION_BITS);
    CHECK(d.duty > 0);
    CHECK(d.duty < pwmFullScale(PWM_RESOLUTION_BITS));
    CHECK(!d.reverse);
}

static void twelve_bits_is_resolvable_at_both_configured_frequencies() {
    const uint32_t f_bus_hz = 150000000u;
    const uint32_t steps = 1u << PWM_RESOLUTION_BITS;
    CHECK(steps <= f_bus_hz / SERVO_PWM_FREQUENCY_HZ);
    CHECK(steps <= f_bus_hz / MOTOR_PWM_FREQUENCY_HZ);
}

static void the_h_bridge_frequency_is_above_the_audible_whine() {
    CHECK(MOTOR_PWM_FREQUENCY_HZ >= 8000);
    CHECK(MOTOR_PWM_FREQUENCY_HZ <= 20000);
}

int main() {
    RUN_TEST(full_scale_matches_the_resolution);
    RUN_TEST(neutral_servo_pulse_is_a_thirteenth_of_a_fifty_hertz_frame);
    RUN_TEST(servo_duty_is_monotonic_across_the_pulse_range);
    RUN_TEST(servo_duty_never_exceeds_full_scale);
    RUN_TEST(a_zero_frequency_is_refused_rather_than_dividing_by_zero);
    RUN_TEST(a_stopped_command_produces_no_duty_at_all);
    RUN_TEST(full_forward_is_full_duty_forward);
    RUN_TEST(full_reverse_is_full_duty_with_the_direction_pin_set);
    RUN_TEST(the_magnitude_is_symmetric_about_zero);
    RUN_TEST(the_deadband_suppresses_a_buzz_but_nothing_above_it);
    RUN_TEST(the_deadband_is_small_enough_to_still_crawl);
    RUN_TEST(duty_is_monotonic_and_bounded_across_the_command_range);
    RUN_TEST(an_out_of_range_effort_is_clamped_not_wrapped);
    RUN_TEST(a_de_energized_drive_stage_reaches_both_output_modes_as_a_stop);
    RUN_TEST(a_commanded_drive_reaches_the_h_bridge_as_duty);
    RUN_TEST(twelve_bits_is_resolvable_at_both_configured_frequencies);
    RUN_TEST(the_h_bridge_frequency_is_above_the_audible_whine);
    return testing::summary("test_motor");
}
