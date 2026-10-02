#include <math.h>

#include "mc_test.h"
#include "motorctl/mc_filter.h"

static void test_lpf_step_response_matches_exponential(void)
{
    const float tau = 5e-3f, ts = 1e-3f;
    mc_lpf1_t f;
    CHECK(mc_lpf1_init(&f, tau, ts, 0.0f) == MC_OK);
    for (int k = 1; k <= 40; ++k) {
        const float y = mc_lpf1_update(&f, 1.0f);
        /* step-invariant discretisation: exact at the sampling instants */
        CHECK_NEAR(y, 1.0 - exp(-k * (double)ts / (double)tau), 1e-6);
    }
}

static void test_lpf_zero_tau_is_pass_through(void)
{
    mc_lpf1_t f;
    CHECK(mc_lpf1_init(&f, 0.0f, 1e-3f, 3.0f) == MC_OK);
    CHECK(mc_lpf1_update(&f, -2.5f) == -2.5f);
    CHECK(mc_lpf1_update(&f, 7.0f) == 7.0f);
}

static void test_lpf_rejects_invalid_arguments(void)
{
    mc_lpf1_t f;
    CHECK(mc_lpf1_init(&f, -1.0f, 1e-3f, 0.0f) == MC_EINVAL);
    CHECK(mc_lpf1_init(&f, 1.0f, 0.0f, 0.0f) == MC_EINVAL);
    CHECK(mc_lpf1_init(&f, NAN, 1e-3f, 0.0f) == MC_EINVAL);
    CHECK(mc_lpf1_init(NULL, 1.0f, 1e-3f, 0.0f) == MC_EINVAL);
}

static void test_slew_ramps_at_rate_and_stops_at_target(void)
{
    mc_slew_t s;
    CHECK(mc_slew_init(&s, 20.0f, 1e-3f, 0.0f) == MC_OK);
    /* 1 rad at 20 rad/s takes 50 samples of 0.02 rad */
    for (int k = 1; k <= 49; ++k) {
        CHECK_NEAR(mc_slew_update(&s, 1.0f), 0.02 * k, 1e-5);
    }
    CHECK_NEAR(mc_slew_update(&s, 1.0f), 1.0, 1e-5);
    for (int k = 0; k < 10; ++k) {
        CHECK_NEAR(mc_slew_update(&s, 1.0f), 1.0, 1e-5); /* no overshoot, no limit cycle */
    }
    CHECK_NEAR(mc_slew_update(&s, -1.0f), 0.98, 1e-5); /* reversal is limited too */
}

static void test_slew_zero_rate_disables_and_rate_can_change(void)
{
    mc_slew_t s;
    CHECK(mc_slew_init(&s, 0.0f, 1e-3f, 0.0f) == MC_OK);
    CHECK(mc_slew_update(&s, 5.0f) == 5.0f);
    mc_slew_set_rate(&s, 1000.0f);
    CHECK_NEAR(mc_slew_update(&s, 0.0f), 4.0, 1e-6);
    mc_slew_set_rate(&s, -3.0f); /* negative = off as well */
    CHECK(mc_slew_update(&s, -8.0f) == -8.0f);
    CHECK(mc_slew_init(&s, 1.0f, 0.0f, 0.0f) == MC_EINVAL);
}

int main(void)
{
    RUN_TEST(test_lpf_step_response_matches_exponential);
    RUN_TEST(test_lpf_zero_tau_is_pass_through);
    RUN_TEST(test_lpf_rejects_invalid_arguments);
    RUN_TEST(test_slew_ramps_at_rate_and_stops_at_target);
    RUN_TEST(test_slew_zero_rate_disables_and_rate_can_change);
    return mc_test_report();
}
