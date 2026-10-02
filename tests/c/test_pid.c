#include <math.h>

#include "golden.h"
#include "mc_test.h"
#include "motorctl/mc_pid.h"

/* float32 vs float64 reference: relative 1e-5 covers accumulated rounding
 * over ~900 samples; the absolute part covers outputs that cross zero. */
#define GOLDEN_ATOL 2e-4
#define GOLDEN_RTOL 1e-5

static mc_pid_config_t config_from_golden(const golden_t *g)
{
    mc_pid_config_t c;
    c.kp = (float)golden_param(g, "kp");
    c.ki = (float)golden_param(g, "ki");
    c.kd = (float)golden_param(g, "kd");
    c.tf = (float)golden_param(g, "tf");
    c.b = (float)golden_param(g, "b");
    c.c = (float)golden_param(g, "c");
    c.ts = (float)golden_param(g, "ts");
    c.u_min = (float)golden_param(g, "u_min");
    c.u_max = (float)golden_param(g, "u_max");
    c.tt = (float)golden_param(g, "tt");
    return c;
}

/* Replays the (r, y) sequence and the events of a golden file and compares
 * the controller output sample by sample. */
static void replay(const char *file)
{
    golden_t g;
    if (!golden_load(file, &g)) {
        CHECK(0);
        return;
    }
    const size_t c_event = golden_column(&g, "event"), c_a0 = golden_column(&g, "a0"),
                 c_a1 = golden_column(&g, "a1"), c_a2 = golden_column(&g, "a2"),
                 c_r = golden_column(&g, "r"), c_y = golden_column(&g, "y"),
                 c_u = golden_column(&g, "u");
    const mc_pid_config_t cfg = config_from_golden(&g);
    mc_pid_t pid;
    CHECK(mc_pid_init(&pid, &cfg) == MC_OK);
    golden_cmp_t cmp = {file, GOLDEN_ATOL, GOLDEN_RTOL, 0.0, 0, 0};
    int events = 0, saturated = 0;
    for (size_t k = 0; k < g.n_rows; ++k) {
        const float r = (float)golden_at(&g, k, c_r), y = (float)golden_at(&g, k, c_y);
        const int event = (int)golden_at(&g, k, c_event);
        if (event == 1) {
            CHECK(mc_pid_set_gains(&pid, (float)golden_at(&g, k, c_a0),
                                   (float)golden_at(&g, k, c_a1), (float)golden_at(&g, k, c_a2), r,
                                   y) == MC_OK);
            ++events;
        } else if (event == 2) {
            mc_pid_bumpless_start(&pid, r, y, (float)golden_at(&g, k, c_a0));
            ++events;
        }
        const float u = mc_pid_update(&pid, r, y);
        saturated += (u == cfg.u_min || u == cfg.u_max);
        golden_cmp_add(&cmp, k, u, golden_at(&g, k, c_u));
    }
    golden_cmp_print(&cmp);
    CHECK(cmp.violations == 0);
    CHECK(events > 0);
    CHECK(saturated > 10); /* the scenario must actually exercise the anti-windup */
    golden_free(&g);
}

static void test_golden_speed_pi(void)
{
    replay("pid_speed_pi.csv");
}

static void test_golden_pid_with_setpoint_weights(void)
{
    replay("pid_position_pid.csv");
}

static void test_rejects_invalid_config(void)
{
    mc_pid_t pid;
    mc_pid_config_t c = mc_pid_default_config(1e-3f);
    CHECK(mc_pid_init(&pid, &c) == MC_OK);
    c.ts = 0.0f;
    CHECK(mc_pid_init(&pid, &c) == MC_EINVAL);
    c = mc_pid_default_config(1e-3f);
    c.u_min = 1.0f;
    c.u_max = 1.0f;
    CHECK(mc_pid_init(&pid, &c) == MC_EINVAL);
    c = mc_pid_default_config(1e-3f);
    c.kd = 0.1f; /* derivative without filter */
    CHECK(mc_pid_init(&pid, &c) == MC_EINVAL);
    c.tf = 1e-3f;
    CHECK(mc_pid_init(&pid, &c) == MC_OK);
    c.kp = NAN;
    CHECK(mc_pid_init(&pid, &c) == MC_EINVAL);
    c = mc_pid_default_config(1e-3f);
    c.tt = -1.0f;
    CHECK(mc_pid_init(&pid, &c) == MC_EINVAL);
}

static void test_integrator_is_trapezoidal(void)
{
    mc_pid_config_t c = mc_pid_default_config(0.01f);
    c.ki = 2.0f;
    mc_pid_t pid;
    CHECK(mc_pid_init(&pid, &c) == MC_OK);
    /* error ramp e[k] = k: trapezoid integration of a ramp is exact,
     * except for the rectangular first step after a reset */
    double expected = 0.0;
    for (int k = 0; k <= 50; ++k) {
        const float u = mc_pid_update(&pid, (float)k, 0.0f);
        expected += k == 0 ? 0.0 : 2.0 * 0.01 * (k - 0.5);
        CHECK_NEAR(u, expected, 1e-4);
    }
}

static void test_derivative_on_measurement_has_no_setpoint_kick(void)
{
    mc_pid_config_t c = mc_pid_default_config(1e-3f);
    c.kp = 2.0f;
    c.kd = 0.5f;
    c.tf = 2e-3f;
    mc_pid_t on_meas, on_error;
    c.c = 0.0f;
    CHECK(mc_pid_init(&on_meas, &c) == MC_OK);
    c.c = 1.0f;
    CHECK(mc_pid_init(&on_error, &c) == MC_OK);
    (void)mc_pid_update(&on_meas, 0.0f, 0.0f);
    (void)mc_pid_update(&on_error, 0.0f, 0.0f);
    /* setpoint step of 1 with constant measurement */
    CHECK_NEAR(mc_pid_update(&on_meas, 1.0f, 0.0f), 2.0, 1e-6);
    /* Tustin filter: first derivative sample is bd * step = 2 / (2 tf + Ts) */
    CHECK_NEAR(mc_pid_update(&on_error, 1.0f, 0.0f), 2.0 + 0.5 * 2.0 / (4e-3 + 1e-3), 1e-3);
}

static void test_filtered_derivative_of_ramp_converges_to_slope(void)
{
    mc_pid_config_t c = mc_pid_default_config(1e-3f);
    c.kd = 1.0f;
    c.tf = 5e-3f;
    mc_pid_t pid;
    CHECK(mc_pid_init(&pid, &c) == MC_OK);
    float u = 0.0f;
    for (int k = 0; k < 200; ++k) {
        u = mc_pid_update(&pid, 0.0f, -0.003f * (float)k); /* d(-y)/dt = 3 */
    }
    CHECK_NEAR(u, 3.0, 1e-3);
}

static void test_back_calculation_bounds_the_integrator(void)
{
    /* Constant error e into a saturated output. In continuous time the
     * integrator settles where  ki e + (u_max - kp e - I) / tt = 0; with the
     * update order of mc_pid_update (integrate, then correct) the discrete
     * fixed point is  I = u_max - kp e + ki e (tt - Ts). */
    const float kp = 1.0f, ki = 50.0f, tt = 0.02f, e = 2.0f, u_max = 1.0f;
    mc_pid_config_t c = mc_pid_default_config(1e-3f);
    c.kp = kp;
    c.ki = ki;
    c.u_min = -u_max;
    c.u_max = u_max;
    c.tt = tt;
    mc_pid_t aw, windup;
    CHECK(mc_pid_init(&aw, &c) == MC_OK);
    c.tt = 0.0f;
    CHECK(mc_pid_init(&windup, &c) == MC_OK);
    for (int k = 0; k < 2000; ++k) {
        CHECK(mc_pid_update(&aw, e, 0.0f) == u_max);
        (void)mc_pid_update(&windup, e, 0.0f);
    }
    CHECK_NEAR(aw.integ, u_max - kp * e + ki * e * (tt - 1e-3), 1e-4);
    CHECK_NEAR(windup.integ, ki * e * 2.0, 0.2); /* grows without bound: 2 s of ki e */
    /* error reverses: the protected controller leaves saturation immediately */
    CHECK(mc_pid_update(&aw, -0.5f, 0.0f) < u_max);
    CHECK(mc_pid_update(&windup, -0.5f, 0.0f) == u_max);
}

static void test_gain_change_is_bumpless(void)
{
    mc_pid_config_t c = mc_pid_default_config(1e-3f);
    c.kp = 2.0f;
    c.ki = 10.0f;
    mc_pid_t pid;
    CHECK(mc_pid_init(&pid, &c) == MC_OK);
    float u0 = 0.0f;
    for (int k = 0; k < 100; ++k) {
        u0 = mc_pid_update(&pid, 1.0f, 0.5f);
    }
    CHECK(mc_pid_set_gains(&pid, 6.0f, 20.0f, 0.0f, 1.0f, 0.5f) == MC_OK);
    const float u1 = mc_pid_update(&pid, 1.0f, 0.5f);
    /* only the regular integration increment, no jump from the kp change */
    CHECK_NEAR(u1 - u0, 20.0 * 1e-3 * 0.5, 1e-5);
    /* switching the integral off clears the integrator instead */
    CHECK(mc_pid_set_gains(&pid, 6.0f, 0.0f, 0.0f, 1.0f, 0.5f) == MC_OK);
    CHECK_NEAR(mc_pid_update(&pid, 1.0f, 0.5f), 3.0, 1e-6);
    /* a derivative gain needs a filter */
    CHECK(mc_pid_set_gains(&pid, 6.0f, 0.0f, 0.1f, 1.0f, 0.5f) == MC_EINVAL);
}

static void test_bumpless_start_continues_from_given_output(void)
{
    mc_pid_config_t c = mc_pid_default_config(1e-3f);
    c.kp = 3.0f;
    c.ki = 40.0f;
    c.b = 0.5f;
    mc_pid_t pid;
    CHECK(mc_pid_init(&pid, &c) == MC_OK);
    mc_pid_bumpless_start(&pid, 2.0f, 2.0f, 4.25f);
    CHECK_NEAR(mc_pid_update(&pid, 2.0f, 2.0f), 4.25, 1e-6);
    /* without integral action the controller starts from its own output */
    c.ki = 0.0f;
    CHECK(mc_pid_init(&pid, &c) == MC_OK);
    mc_pid_bumpless_start(&pid, 2.0f, 2.0f, 4.25f);
    CHECK_NEAR(mc_pid_update(&pid, 2.0f, 2.0f), 3.0 * (0.5 * 2.0 - 2.0), 1e-6);
}

int main(void)
{
    RUN_TEST(test_golden_speed_pi);
    RUN_TEST(test_golden_pid_with_setpoint_weights);
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_integrator_is_trapezoidal);
    RUN_TEST(test_derivative_on_measurement_has_no_setpoint_kick);
    RUN_TEST(test_filtered_derivative_of_ramp_converges_to_slope);
    RUN_TEST(test_back_calculation_bounds_the_integrator);
    RUN_TEST(test_gain_change_is_bumpless);
    RUN_TEST(test_bumpless_start_continues_from_given_output);
    return mc_test_report();
}
