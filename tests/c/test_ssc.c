#include <math.h>

#include "golden.h"
#include "mc_test.h"
#include "motorctl/mc_ssc.h"
#include "servo_config.h"

/* The integrator gain is large (ki ~ 3e3 V/(rad s)), so float32 rounding of
 * the integrator state shows up amplified in the output. */
#define GOLDEN_ATOL 1e-3
#define GOLDEN_RTOL 1e-4

static void test_golden_replay(void)
{
    golden_t g;
    if (!golden_load("ssc.csv", &g)) {
        CHECK(0);
        return;
    }
    mc_ssc_config_t c;
    c.phi[0][0] = (float)golden_param(&g, "phi00");
    c.phi[0][1] = (float)golden_param(&g, "phi01");
    c.phi[1][0] = (float)golden_param(&g, "phi10");
    c.phi[1][1] = (float)golden_param(&g, "phi11");
    c.gamma[0] = (float)golden_param(&g, "gamma0");
    c.gamma[1] = (float)golden_param(&g, "gamma1");
    c.l[0] = (float)golden_param(&g, "l0");
    c.l[1] = (float)golden_param(&g, "l1");
    c.k[0] = (float)golden_param(&g, "k0");
    c.k[1] = (float)golden_param(&g, "k1");
    c.ki = (float)golden_param(&g, "ki");
    c.kr = (float)golden_param(&g, "kr");
    c.ts = (float)golden_param(&g, "ts");
    c.u_min = (float)golden_param(&g, "u_min");
    c.u_max = (float)golden_param(&g, "u_max");
    c.tt = (float)golden_param(&g, "tt");
    const size_t c_event = golden_column(&g, "event"), c_a0 = golden_column(&g, "a0"),
                 c_a1 = golden_column(&g, "a1"), c_a2 = golden_column(&g, "a2"),
                 c_r = golden_column(&g, "r"), c_y = golden_column(&g, "y"),
                 c_u = golden_column(&g, "u");
    mc_ssc_t s;
    CHECK(mc_ssc_init(&s, &c) == MC_OK);
    golden_cmp_t cmp = {"ssc.csv", GOLDEN_ATOL, GOLDEN_RTOL, 0.0, 0, 0};
    int saturated = 0, restarts = 0;
    for (size_t k = 0; k < g.n_rows; ++k) {
        const float r = (float)golden_at(&g, k, c_r), y = (float)golden_at(&g, k, c_y);
        if ((int)golden_at(&g, k, c_event) == 2) {
            mc_ssc_bumpless_start(&s, r, (float)golden_at(&g, k, c_a0),
                                  (float)golden_at(&g, k, c_a1), (float)golden_at(&g, k, c_a2));
            ++restarts;
        }
        const float u = mc_ssc_update(&s, r, y);
        saturated += (u == c.u_min || u == c.u_max);
        golden_cmp_add(&cmp, k, u, golden_at(&g, k, c_u));
    }
    golden_cmp_print(&cmp);
    CHECK(cmp.violations == 0);
    CHECK(restarts == 1);
    CHECK(saturated > 10);
    golden_free(&g);
}

/* Closed loop with the controller's own design model as the plant, in float. */
typedef struct {
    float x[2];
} model_t;

/* Advances the model by one sample and returns the new measurement y[k+1]. */
static float model_step(model_t *m, const mc_ssc_config_t *c, float u)
{
    const float x0 = c->phi[0][0] * m->x[0] + c->phi[0][1] * m->x[1] + c->gamma[0] * u;
    const float x1 = c->phi[1][0] * m->x[0] + c->phi[1][1] * m->x[1] + c->gamma[1] * u;
    m->x[0] = x0;
    m->x[1] = x1;
    return x0;
}

static void test_observer_error_decays_with_placed_poles(void)
{
    const mc_servo_config_t sc = servo_config_from_params();
    mc_ssc_t s;
    CHECK(mc_ssc_init(&s, &sc.ssc) == MC_OK);
    mc_ssc_reset(&s, 0.0f);
    model_t m = {{0.05f, 5.0f}}; /* moving at 5 rad/s, observer thinks 0 */
    float y = m.x[0];
    for (int k = 0; k < 60; ++k) {
        const float u = mc_ssc_update(&s, 0.0f, y);
        y = model_step(&m, &sc.ssc, u);
    }
    /* Separation principle: e[k+1] = (Phi - l c) e[k] regardless of u. The
     * observer poles are exp(-300 Ts) and exp(-360 Ts); after 60 samples the
     * initial error of 5 rad/s has decayed by more than 1e-6. */
    CHECK_NEAR(s.xhat[0], m.x[0], 1e-5);
    CHECK_NEAR(s.xhat[1], m.x[1], 1e-4);
}

static void test_closed_loop_reaches_setpoint_without_offset(void)
{
    const mc_servo_config_t sc = servo_config_from_params();
    mc_ssc_t s;
    CHECK(mc_ssc_init(&s, &sc.ssc) == MC_OK);
    model_t m = {{0.0f, 0.0f}};
    float y = 0.0f, u = 0.0f;
    for (int k = 0; k < 1000; ++k) {
        u = mc_ssc_update(&s, 0.2f, y);
        /* constant input disturbance (load torque referred to voltage) */
        y = model_step(&m, &sc.ssc, u - 1.5f);
    }
    CHECK_NEAR(m.x[0], 0.2, 1e-4); /* integral action removes the offset */
    CHECK_NEAR(m.x[1], 0.0, 1e-3);
    CHECK_NEAR(u, 1.5, 1e-2);      /* ... by supplying the disturbance voltage */
}

static float step_overshoot(float tt)
{
    mc_servo_config_t sc = servo_config_from_params();
    sc.ssc.tt = tt;
    mc_ssc_t s;
    CHECK(mc_ssc_init(&s, &sc.ssc) == MC_OK);
    model_t m = {{0.0f, 0.0f}};
    float y = 0.0f, peak = 0.0f;
    /* 5 s: without anti-windup the loop still settles, but only after a
     * large excursion that takes well over a second to die out. */
    for (int k = 0; k < 5000; ++k) {
        const float u = mc_ssc_update(&s, 3.0f, y);
        y = model_step(&m, &sc.ssc, u);
        peak = fmaxf(peak, y);
    }
    CHECK_NEAR(m.x[0], 3.0, 1e-3);
    return (peak - 3.0f) / 3.0f;
}

static void test_back_calculation_prevents_windup(void)
{
    /* A 3 rad step saturates the 12 V output for tens of milliseconds. */
    const float with_aw = step_overshoot(servo_config_from_params().ssc.tt);
    const float without = step_overshoot(0.0f);
    printf("  overshoot with / without back-calculation: %.4f / %.4f\n", (double)with_aw,
           (double)without);
    CHECK(with_aw < 0.05f);
    CHECK(without > 0.5f);
}

static void test_bumpless_start_continues_output(void)
{
    const mc_servo_config_t sc = servo_config_from_params();
    mc_ssc_t s;
    CHECK(mc_ssc_init(&s, &sc.ssc) == MC_OK);
    mc_ssc_bumpless_start(&s, 1.0f, 1.0f, 4.0f, 6.5f);
    CHECK_NEAR(mc_ssc_update(&s, 1.0f, 1.0f), 6.5, 1e-4);
}

static void test_rejects_invalid_config(void)
{
    const mc_servo_config_t sc = servo_config_from_params();
    mc_ssc_config_t c = sc.ssc;
    mc_ssc_t s;
    c.phi[1][1] = NAN;
    CHECK(mc_ssc_init(&s, &c) == MC_EINVAL);
    c = sc.ssc;
    c.u_min = c.u_max;
    CHECK(mc_ssc_init(&s, &c) == MC_EINVAL);
    c = sc.ssc;
    c.tt = -1.0f;
    CHECK(mc_ssc_init(&s, &c) == MC_EINVAL);
    c = sc.ssc;
    c.ts = 0.0f;
    CHECK(mc_ssc_init(&s, &c) == MC_EINVAL);
}

int main(void)
{
    RUN_TEST(test_golden_replay);
    RUN_TEST(test_observer_error_decays_with_placed_poles);
    RUN_TEST(test_closed_loop_reaches_setpoint_without_offset);
    RUN_TEST(test_back_calculation_prevents_windup);
    RUN_TEST(test_bumpless_start_continues_output);
    RUN_TEST(test_rejects_invalid_config);
    return mc_test_report();
}
