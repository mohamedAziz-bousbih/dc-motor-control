#include "motorctl/mc_cmd.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TOKENS 6

static bool keyword_eq(const char *token, const char *keyword)
{
    while (*token != '\0' && *keyword != '\0') {
        if (tolower((unsigned char)*token) != *keyword) {
            return false;
        }
        ++token;
        ++keyword;
    }
    return *token == '\0' && *keyword == '\0';
}

/* Splits buf in place at whitespace. Returns the token count, or
 * MAX_TOKENS + 1 if there are more tokens than any command takes. */
static size_t tokenize(char *buf, char *tokens[MAX_TOKENS])
{
    size_t n = 0;
    char *p = buf;
    for (;;) {
        while (*p != '\0' && isspace((unsigned char)*p)) {
            ++p;
        }
        if (*p == '\0') {
            return n;
        }
        if (n == MAX_TOKENS) {
            return MAX_TOKENS + 1;
        }
        tokens[n++] = p;
        while (*p != '\0' && !isspace((unsigned char)*p)) {
            ++p;
        }
        if (*p != '\0') {
            *p++ = '\0';
        }
    }
}

/* Whole token must be a number; strtof accepts "inf"/"nan", which the
 * callers reject as out of range rather than as malformed. */
static mc_parse_result_t parse_float(const char *token, float *out)
{
    char *end = NULL;
    const float v = strtof(token, &end);
    if (end == token || *end != '\0') {
        return MC_PARSE_BAD_ARGS;
    }
    if (!isfinite(v)) {
        return MC_PARSE_OUT_OF_RANGE;
    }
    *out = v;
    return MC_PARSE_OK;
}

static bool parse_loop(const char *token, mc_loop_t *loop)
{
    if (keyword_eq(token, "speed")) {
        *loop = MC_LOOP_SPEED;
        return true;
    }
    if (keyword_eq(token, "pos")) {
        *loop = MC_LOOP_POSITION;
        return true;
    }
    return false;
}

static mc_parse_result_t parse_mode(char *tokens[], size_t n, mc_cmd_t *cmd)
{
    static const struct {
        const char *name;
        mc_mode_t mode;
    } modes[] = {{"off", MC_MODE_OFF},
                 {"volt", MC_MODE_VOLTAGE},
                 {"speed", MC_MODE_SPEED},
                 {"pos", MC_MODE_POSITION},
                 {"state", MC_MODE_STATE}};
    if (n != 2) {
        return MC_PARSE_BAD_ARGS;
    }
    for (size_t i = 0; i < sizeof modes / sizeof modes[0]; ++i) {
        if (keyword_eq(tokens[1], modes[i].name)) {
            cmd->kind = MC_CMD_MODE;
            cmd->mode = modes[i].mode;
            return MC_PARSE_OK;
        }
    }
    return MC_PARSE_UNKNOWN;
}

static mc_parse_result_t parse_gains(char *tokens[], size_t n, mc_cmd_t *cmd)
{
    if (n != 5) {
        return MC_PARSE_BAD_ARGS;
    }
    if (!parse_loop(tokens[1], &cmd->loop)) {
        return MC_PARSE_UNKNOWN;
    }
    for (size_t i = 0; i < 3; ++i) {
        const mc_parse_result_t r = parse_float(tokens[2 + i], &cmd->value[i]);
        if (r != MC_PARSE_OK) {
            return r;
        }
        if (cmd->value[i] < 0.0f) {
            return MC_PARSE_OUT_OF_RANGE; /* negative gains would give positive feedback */
        }
    }
    cmd->kind = MC_CMD_GAINS;
    return MC_PARSE_OK;
}

static mc_parse_result_t parse_slew(char *tokens[], size_t n, mc_cmd_t *cmd)
{
    if (n != 3) {
        return MC_PARSE_BAD_ARGS;
    }
    if (!parse_loop(tokens[1], &cmd->loop)) {
        return MC_PARSE_UNKNOWN;
    }
    const mc_parse_result_t r = parse_float(tokens[2], &cmd->value[0]);
    if (r != MC_PARSE_OK) {
        return r;
    }
    if (cmd->value[0] < 0.0f) {
        return MC_PARSE_OUT_OF_RANGE;
    }
    cmd->kind = MC_CMD_SLEW;
    return MC_PARSE_OK;
}

static mc_parse_result_t parse_tele(char *tokens[], size_t n, mc_cmd_t *cmd)
{
    if (n != 2) {
        return MC_PARSE_BAD_ARGS;
    }
    const char *t = tokens[1];
    if (*t == '\0' || !isdigit((unsigned char)*t)) {
        return MC_PARSE_BAD_ARGS;
    }
    char *end = NULL;
    const unsigned long v = strtoul(t, &end, 10);
    if (*end != '\0') {
        return MC_PARSE_BAD_ARGS;
    }
    if (v > 100000ul) {
        return MC_PARSE_OUT_OF_RANGE;
    }
    cmd->kind = MC_CMD_TELEMETRY;
    cmd->divider = (uint32_t)v;
    return MC_PARSE_OK;
}

mc_parse_result_t mc_cmd_parse(const char *line, mc_cmd_t *cmd)
{
    char buf[MC_CMD_MAX_LINE + 1];
    char *tokens[MAX_TOKENS];
    const size_t len = strlen(line);
    if (len > MC_CMD_MAX_LINE) {
        return MC_PARSE_BAD_ARGS;
    }
    memcpy(buf, line, len + 1);
    char *hash = strchr(buf, '#');
    if (hash != NULL) {
        *hash = '\0';
    }
    const size_t n = tokenize(buf, tokens);
    if (n == 0) {
        return MC_PARSE_EMPTY;
    }
    if (n > MAX_TOKENS) {
        return MC_PARSE_BAD_ARGS;
    }
    memset(cmd, 0, sizeof *cmd);

    if (keyword_eq(tokens[0], "mode")) {
        return parse_mode(tokens, n, cmd);
    }
    if (keyword_eq(tokens[0], "sp")) {
        if (n != 2) {
            return MC_PARSE_BAD_ARGS;
        }
        cmd->kind = MC_CMD_SETPOINT;
        return parse_float(tokens[1], &cmd->value[0]);
    }
    if (keyword_eq(tokens[0], "gains")) {
        return parse_gains(tokens, n, cmd);
    }
    if (keyword_eq(tokens[0], "slew")) {
        return parse_slew(tokens, n, cmd);
    }
    if (keyword_eq(tokens[0], "tele")) {
        return parse_tele(tokens, n, cmd);
    }
    if (keyword_eq(tokens[0], "status") || keyword_eq(tokens[0], "help")) {
        if (n != 1) {
            return MC_PARSE_BAD_ARGS;
        }
        cmd->kind = keyword_eq(tokens[0], "status") ? MC_CMD_STATUS : MC_CMD_HELP;
        return MC_PARSE_OK;
    }
    return MC_PARSE_UNKNOWN;
}

const char *mc_parse_result_str(mc_parse_result_t r)
{
    switch (r) {
    case MC_PARSE_OK:
        return "ok";
    case MC_PARSE_EMPTY:
        return "empty line";
    case MC_PARSE_UNKNOWN:
        return "unknown command or keyword";
    case MC_PARSE_BAD_ARGS:
        return "wrong arguments";
    case MC_PARSE_OUT_OF_RANGE:
        return "value out of range";
    default:
        return "?";
    }
}

mc_status_t mc_cmd_apply(const mc_cmd_t *cmd, mc_servo_t *servo)
{
    switch (cmd->kind) {
    case MC_CMD_MODE:
        return mc_servo_set_mode(servo, cmd->mode);
    case MC_CMD_SETPOINT:
        mc_servo_set_setpoint(servo, cmd->value[0]);
        return MC_OK;
    case MC_CMD_GAINS:
        return mc_servo_set_pid_gains(servo, cmd->loop, cmd->value[0], cmd->value[1],
                                      cmd->value[2]);
    case MC_CMD_SLEW:
        return mc_servo_set_slew(servo, cmd->loop, cmd->value[0]);
    case MC_CMD_TELEMETRY:
    case MC_CMD_STATUS:
    case MC_CMD_HELP:
    default:
        return MC_EINVAL;
    }
}
