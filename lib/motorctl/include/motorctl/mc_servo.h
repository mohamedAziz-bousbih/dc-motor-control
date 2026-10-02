/*
 * Servo module: everything that happens in one control period of the
 * firmware, independent of the hardware.
 * Reference model: Servo in control/servo.py (statement by statement).
 *
 *   encoder counts -> angle, low-pass filtered difference-quotient speed
 *                  -> setpoint rate limiter -> active controller -> voltage
 *
 * Modes: OFF (0 V), VOLTAGE (open loop, for identification experiments),
 * SPEED (PI), POSITION (PD, pole compensation), STATE (state feedback with
 * observer). Switching into a closed-loop mode holds the current speed or
 * angle and continues from the last output where the controller allows it.
 */
#ifndef MOTORCTL_MC_SERVO_H
#define MOTORCTL_MC_SERVO_H

#include <stdint.h>

#include "motorctl/mc_common.h"
#include "motorctl/mc_filter.h"
#include "motorctl/mc_pid.h"
#include "motorctl/mc_ssc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MC_MODE_OFF = 0,
    MC_MODE_VOLTAGE = 1,  /* setpoint in V */
    MC_MODE_SPEED = 2,    /* setpoint in rad/s */
    MC_MODE_POSITION = 3, /* setpoint in rad */
    MC_MODE_STATE = 4     /* setpoint in rad */
} mc_mode_t;

typedef enum {
    MC_LOOP_SPEED = 0,
    MC_LOOP_POSITION = 1 /* PD and state mode share the position setpoint limiter */
} mc_loop_t;

typedef struct {
    float ts;              /* sample time [s] */
    float rad_per_count;   /* encoder resolution at the output shaft [rad] */
    float v_max;           /* actuator limit = H-bridge supply [V] */
    mc_pid_config_t speed;
    float speed_lpf_tau;   /* low-pass on the difference-quotient speed [s] */
    float speed_slew;      /* speed setpoint rate limit [rad/s^2], 0 = off */
    mc_pid_config_t pos;
    float pos_slew;        /* position setpoint rate limit [rad/s], 0 = off */
    mc_ssc_config_t ssc;
} mc_servo_config_t;

typedef struct {
    mc_servo_config_t cfg;
    mc_pid_t speed_pid;
    mc_pid_t pos_pid;
    mc_ssc_t ssc;
    mc_lpf1_t lpf;
    mc_slew_t slew;
    float speed_slew;      /* current rate limits (changeable at run time) */
    float pos_slew;
    mc_mode_t mode;
    int32_t counts_prev;
    float theta;           /* measured angle [rad] */
    float omega;           /* filtered measured speed [rad/s] */
    float target;          /* commanded setpoint */
    float r_eff;           /* setpoint after the rate limiter */
    float u;               /* last output [V] */
    uint32_t nonfinite_trips; /* times a NaN/inf output forced MC_MODE_OFF */
} mc_servo_t;

mc_status_t mc_servo_init(mc_servo_t *s, const mc_servo_config_t *cfg, int32_t counts0);
mc_status_t mc_servo_set_mode(mc_servo_t *s, mc_mode_t mode);
void mc_servo_set_setpoint(mc_servo_t *s, float value);
mc_status_t mc_servo_set_pid_gains(mc_servo_t *s, mc_loop_t loop, float kp, float ki, float kd);
mc_status_t mc_servo_set_slew(mc_servo_t *s, mc_loop_t loop, float rate);

/* One control period. counts is the extended (32-bit) encoder position.
 * A non-finite controller output switches the servo off (output 0 V) and
 * increments nonfinite_trips. */
float mc_servo_update(mc_servo_t *s, int32_t counts);

/* Observer speed estimate in state mode, otherwise the filtered measurement. */
float mc_servo_omega_hat(const mc_servo_t *s);

const char *mc_mode_name(mc_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* MOTORCTL_MC_SERVO_H */
