/*
 * Text command interface of the firmware (one command per line).
 *
 *   mode off|volt|speed|pos|state     switch controller (bumpless)
 *   sp <value>                        setpoint: V, rad/s or rad, depending on mode
 *   gains speed|pos <kp> <ki> <kd>    change PID gains (bumpless)
 *   slew speed|pos <rate>             setpoint rate limit, 0 = off
 *   tele <n>                          CSV telemetry every n-th sample, 0 = off
 *   status                            print mode, setpoint and gains
 *   help                              print this list
 *
 * Parsing is separate from the hardware so it can be unit-tested on the
 * host; mc_cmd_apply() executes the commands that act on the servo.
 */
#ifndef MOTORCTL_MC_CMD_H
#define MOTORCTL_MC_CMD_H

#include <stdint.h>

#include "motorctl/mc_servo.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MC_CMD_MAX_LINE 96u /* longer lines are rejected */

typedef enum {
    MC_CMD_MODE,
    MC_CMD_SETPOINT,
    MC_CMD_GAINS,
    MC_CMD_SLEW,
    MC_CMD_TELEMETRY,
    MC_CMD_STATUS,
    MC_CMD_HELP
} mc_cmd_kind_t;

typedef struct {
    mc_cmd_kind_t kind;
    mc_mode_t mode;   /* MC_CMD_MODE */
    mc_loop_t loop;   /* MC_CMD_GAINS, MC_CMD_SLEW */
    float value[3];   /* setpoint | kp, ki, kd | rate */
    uint32_t divider; /* MC_CMD_TELEMETRY */
} mc_cmd_t;

typedef enum {
    MC_PARSE_OK = 0,
    MC_PARSE_EMPTY,       /* blank line or comment (#) */
    MC_PARSE_UNKNOWN,     /* unknown command or keyword */
    MC_PARSE_BAD_ARGS,    /* wrong number of arguments or not a number */
    MC_PARSE_OUT_OF_RANGE /* number not finite, negative gain/rate, ... */
} mc_parse_result_t;

mc_parse_result_t mc_cmd_parse(const char *line, mc_cmd_t *cmd);
const char *mc_parse_result_str(mc_parse_result_t r);

/* Applies MODE, SETPOINT, GAINS and SLEW to the servo. Returns MC_EINVAL for
 * the other kinds and when the servo rejects the values. */
mc_status_t mc_cmd_apply(const mc_cmd_t *cmd, mc_servo_t *servo);

#ifdef __cplusplus
}
#endif

#endif /* MOTORCTL_MC_CMD_H */
