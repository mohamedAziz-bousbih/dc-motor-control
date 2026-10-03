#include <limits.h>
#include <math.h>
#include <stdint.h>

#include "golden.h"
#include "mc_test.h"
#include "motorctl/mc_cmd.h"
#include "motorctl/mc_servo.h"
#include "servo_config.h"

/* Replays tests/golden/servo.csv: the Python reference model of the servo
 * ran in closed loop with the true third-order plant through every mode,
 * bumpless transfers, a gain change and a load step. Here the recorded
 * encoder counts and commands are fed to the C servo built from the same
 * header the firmware uses, and its outputs are compared sample by sample. */
static void test_golden_replay_all_modes(void)
{
    golden_t g;
    if (!golden_load("servo.csv", &g)) {
        CHECK(0);
        return;
    }
    const size_t c_cmd = golden_column(&g, "cmd"), c_a0 = golden_column(&g, "a0"),
                 c_a1 = golden_column(&g, "a1"), c_a2 = golden_column(&g, "a2"),
                 c_a3 = golden_column(&g, "a3"), c_counts = golden_column(&g, "counts"),
                 c_u = golden_column(&g, "u"), c_r = golden_column(&g, "r"),
                 c_omega = golden_column(&g, "omega");
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    CHECK(mc_servo_init(&s, &cfg, 0) == MC_OK);
    golden_cmp_t cu = {"servo.csv u [V]", 2e-3, 1e-4, 0.0, 0, 0};
    golden_cmp_t cr = {"servo.csv r", 1e-4, 1e-5, 0.0, 0, 0};
    golden_cmp_t cw = {"servo.csv omega [rad/s]", 1e-4, 1e-5, 0.0, 0, 0};
    int visited[5] = {0};
    for (size_t k = 0; k < g.n_rows; ++k) {
        const int code = (int)golden_at(&g, k, c_cmd);
        if (code != 0) {
            mc_cmd_t cmd = {0};
            const float a0 = (float)golden_at(&g, k, c_a0);
            switch (code) {
            case 1:
                cmd.kind = MC_CMD_MODE;
                cmd.mode = (mc_mode_t)(int)a0;
                break;
            case 2:
                cmd.kind = MC_CMD_SETPOINT;
                cmd.value[0] = a0;
                break;
            case 3:
                cmd.kind = MC_CMD_GAINS;
                cmd.loop = (mc_loop_t)(int)a0;
                cmd.value[0] = (float)golden_at(&g, k, c_a1);
                cmd.value[1] = (float)golden_at(&g, k, c_a2);
                cmd.value[2] = (float)golden_at(&g, k, c_a3);
                break;
            default:
                cmd.kind = MC_CMD_SLEW;
                cmd.loop = (mc_loop_t)(int)a0;
                cmd.value[0] = (float)golden_at(&g, k, c_a1);
                break;
            }
            CHECK(mc_cmd_apply(&cmd, &s) == MC_OK);
        }
        const float u = mc_servo_update(&s, (int32_t)golden_at(&g, k, c_counts));
        visited[s.mode] = 1;
        golden_cmp_add(&cu, k, u, golden_at(&g, k, c_u));
        golden_cmp_add(&cr, k, s.r_eff, golden_at(&g, k, c_r));
        golden_cmp_add(&cw, k, s.omega, golden_at(&g, k, c_omega));
    }
    golden_cmp_print(&cu);
    golden_cmp_print(&cr);
    golden_cmp_print(&cw);
    CHECK(cu.violations == 0);
    CHECK(cr.violations == 0);
    CHECK(cw.violations == 0);
    for (int m = 0; m < 5; ++m) {
        CHECK(visited[m]); /* the scenario covers every mode */
    }
    golden_free(&g);
}

static void test_speed_from_constant_count_rate(void)
{
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    CHECK(mc_servo_init(&s, &cfg, 100) == MC_OK);
    int32_t counts = 100;
    for (int k = 0; k < 200; ++k) {
        counts += 3;
        (void)mc_servo_update(&s, counts);
    }
    CHECK_NEAR(s.omega, 3.0 * cfg.rad_per_count / cfg.ts, 1e-4);
    CHECK_NEAR(s.theta, counts * (double)cfg.rad_per_count, 1e-5);
    CHECK(mc_servo_omega_hat(&s) == s.omega); /* not in state mode */
}

static void test_count_wraparound_is_a_small_step(void)
{
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    CHECK(mc_servo_init(&s, &cfg, INT32_MAX) == MC_OK);
    (void)mc_servo_update(&s, INT32_MIN); /* +1 count across the 32-bit wrap */
    CHECK(s.omega > 0.0f);
    CHECK(s.omega < 1.1f * cfg.rad_per_count / cfg.ts);
}

static void test_voltage_and_off_modes(void)
{
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    CHECK(mc_servo_init(&s, &cfg, 0) == MC_OK);
    CHECK(mc_servo_update(&s, 0) == 0.0f); /* starts in OFF */
    CHECK(mc_servo_set_mode(&s, MC_MODE_VOLTAGE) == MC_OK);
    mc_servo_set_setpoint(&s, 3.5f);
    CHECK(mc_servo_update(&s, 0) == 3.5f);
    mc_servo_set_setpoint(&s, -100.0f); /* clamped to the supply */
    CHECK(mc_servo_update(&s, 0) == -cfg.v_max);
    CHECK(mc_servo_set_mode(&s, MC_MODE_OFF) == MC_OK);
    CHECK(mc_servo_update(&s, 0) == 0.0f);
    CHECK(mc_servo_set_mode(&s, (mc_mode_t)7) == MC_EINVAL);
    CHECK(s.mode == MC_MODE_OFF);
}

static void test_position_mode_holds_angle_at_switch(void)
{
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    CHECK(mc_servo_init(&s, &cfg, 0) == MC_OK);
    (void)mc_servo_update(&s, 250);
    CHECK(mc_servo_set_mode(&s, MC_MODE_POSITION) == MC_OK);
    CHECK_NEAR(s.target, 250 * (double)cfg.rad_per_count, 1e-6);
    /* at the held angle and at rest, the PD output is zero */
    CHECK_NEAR(mc_servo_update(&s, 250), 0.0, 1e-4);
    /* a new setpoint is approached at the slew rate */
    mc_servo_set_setpoint(&s, s.theta + 1.0f);
    (void)mc_servo_update(&s, 250);
    CHECK_NEAR(s.r_eff - s.theta, cfg.pos_slew * cfg.ts, 1e-5);
    CHECK(mc_servo_set_slew(&s, MC_LOOP_POSITION, 0.0f) == MC_OK); /* takes effect at once */
    (void)mc_servo_update(&s, 250);
    CHECK_NEAR(s.r_eff, s.target, 1e-6);
}

/* Runs VOLTAGE mode at 4 V with the motor turning at a constant count rate
 * until the speed filter has settled, then switches to `mode` and returns
 * the first closed-loop output. */
static float output_after_switch_from_voltage(mc_mode_t mode)
{
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    CHECK(mc_servo_init(&s, &cfg, 0) == MC_OK);
    CHECK(mc_servo_set_mode(&s, MC_MODE_VOLTAGE) == MC_OK);
    mc_servo_set_setpoint(&s, 4.0f);
    int32_t counts = 0;
    for (int k = 0; k < 200; ++k) {
        counts += 5;
        CHECK(mc_servo_update(&s, counts) == 4.0f);
    }
    CHECK(mc_servo_set_mode(&s, mode) == MC_OK);
    counts += 5;
    return mc_servo_update(&s, counts);
}

static void test_switch_from_voltage_mode_is_bumpless(void)
{
    /* PI and state controller take over the last output (their integrators
     * absorb it); only the regular one-sample increment may remain. */
    CHECK_NEAR(output_after_switch_from_voltage(MC_MODE_SPEED), 4.0, 1e-3);
    CHECK_NEAR(output_after_switch_from_voltage(MC_MODE_STATE), 4.0, 1e-3);
}

static void test_state_mode_reports_observer_speed(void)
{
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    CHECK(mc_servo_init(&s, &cfg, 0) == MC_OK);
    CHECK(mc_servo_set_mode(&s, MC_MODE_STATE) == MC_OK);
    (void)mc_servo_update(&s, 0);
    (void)mc_servo_update(&s, 2);
    CHECK(mc_servo_omega_hat(&s) == s.ssc.xhat[1]);
    CHECK(mc_servo_set_pid_gains(&s, (mc_loop_t)5, 1.0f, 0.0f, 0.0f) == MC_EINVAL);
}

static void test_nonfinite_output_switches_the_servo_off(void)
{
    /* Reachable from the serial interface: "mode pos", "slew pos 0",
     * "sp 3e38", then "sp 0". The first jump overflows the filtered
     * derivative to +inf (the output just saturates), the second one makes
     * it inf - inf = NaN. mc_clampf passes NaN through, so without the check
     * in mc_servo_update the NaN would reach the H-bridge. */
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    CHECK(mc_servo_init(&s, &cfg, 0) == MC_OK);
    CHECK(mc_servo_set_mode(&s, MC_MODE_POSITION) == MC_OK);
    CHECK(mc_servo_set_slew(&s, MC_LOOP_POSITION, 0.0f) == MC_OK);
    mc_servo_set_setpoint(&s, 3e38f);
    CHECK(mc_servo_update(&s, 0) == cfg.v_max);
    CHECK(s.mode == MC_MODE_POSITION && s.nonfinite_trips == 0);
    mc_servo_set_setpoint(&s, 0.0f);
    const float u = mc_servo_update(&s, 0);
    CHECK(u == 0.0f);
    CHECK(s.u == 0.0f);
    CHECK(s.mode == MC_MODE_OFF);
    CHECK(s.nonfinite_trips == 1);
    /* re-entering a closed-loop mode starts from clean states */
    CHECK(mc_servo_set_mode(&s, MC_MODE_POSITION) == MC_OK);
    CHECK_NEAR(mc_servo_update(&s, 0), 0.0, 1e-6);
}

int main(void)
{
    RUN_TEST(test_golden_replay_all_modes);
    RUN_TEST(test_speed_from_constant_count_rate);
    RUN_TEST(test_count_wraparound_is_a_small_step);
    RUN_TEST(test_voltage_and_off_modes);
    RUN_TEST(test_position_mode_holds_angle_at_switch);
    RUN_TEST(test_switch_from_voltage_mode_is_bumpless);
    RUN_TEST(test_state_mode_reports_observer_speed);
    RUN_TEST(test_nonfinite_output_switches_the_servo_off);
    return mc_test_report();
}
