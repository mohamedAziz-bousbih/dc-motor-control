/*
 * Builds the servo configuration from the generated controller_params.h.
 *
 * Shared by the firmware and by the host test tests/c/test_servo.c, so the
 * golden-vector test exercises exactly the configuration that is flashed.
 */
#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include "controller_params.h"
#include "motorctl/mc_servo.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline mc_servo_config_t servo_config_from_params(void)
{
    mc_servo_config_t c;
    c.ts = CTRL_TS_S;
    c.rad_per_count = CTRL_RAD_PER_COUNT;
    c.v_max = CTRL_V_SUPPLY;

    c.speed = mc_pid_default_config(CTRL_TS_S);
    c.speed.kp = SPEED_KP;
    c.speed.ki = SPEED_KI;
    c.speed.kd = SPEED_KD;
    c.speed.tf = SPEED_TF;
    c.speed.b = SPEED_B;
    c.speed.c = SPEED_C;
    c.speed.tt = SPEED_TT;
    c.speed.u_min = -CTRL_V_SUPPLY;
    c.speed.u_max = CTRL_V_SUPPLY;
    c.speed_lpf_tau = SPEED_LPF_TAU;
    c.speed_slew = SPEED_SLEW;

    c.pos = mc_pid_default_config(CTRL_TS_S);
    c.pos.kp = POS_KP;
    c.pos.ki = POS_KI;
    c.pos.kd = POS_KD;
    c.pos.tf = POS_TF;
    c.pos.b = POS_B;
    c.pos.c = POS_C;
    c.pos.tt = POS_TT;
    c.pos.u_min = -CTRL_V_SUPPLY;
    c.pos.u_max = CTRL_V_SUPPLY;
    c.pos_slew = POS_SLEW;

    c.ssc.phi[0][0] = SSC_PHI00;
    c.ssc.phi[0][1] = SSC_PHI01;
    c.ssc.phi[1][0] = SSC_PHI10;
    c.ssc.phi[1][1] = SSC_PHI11;
    c.ssc.gamma[0] = SSC_GAMMA0;
    c.ssc.gamma[1] = SSC_GAMMA1;
    c.ssc.l[0] = SSC_L0;
    c.ssc.l[1] = SSC_L1;
    c.ssc.k[0] = SSC_K0;
    c.ssc.k[1] = SSC_K1;
    c.ssc.ki = SSC_KI;
    c.ssc.ts = CTRL_TS_S;
    c.ssc.kr = SSC_KR;
    c.ssc.u_min = -CTRL_V_SUPPLY;
    c.ssc.u_max = CTRL_V_SUPPLY;
    c.ssc.tt = SSC_TT;
    return c;
}

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONFIG_H */
