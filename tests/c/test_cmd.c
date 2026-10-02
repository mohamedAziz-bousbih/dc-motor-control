#include <string.h>

#include "mc_test.h"
#include "motorctl/mc_cmd.h"
#include "servo_config.h"

static mc_parse_result_t parse(const char *line, mc_cmd_t *cmd)
{
    return mc_cmd_parse(line, cmd);
}

static void test_parses_every_command(void)
{
    mc_cmd_t c;
    CHECK(parse("mode speed", &c) == MC_PARSE_OK && c.kind == MC_CMD_MODE &&
          c.mode == MC_MODE_SPEED);
    CHECK(parse("mode off", &c) == MC_PARSE_OK && c.mode == MC_MODE_OFF);
    CHECK(parse("mode volt", &c) == MC_PARSE_OK && c.mode == MC_MODE_VOLTAGE);
    CHECK(parse("mode pos", &c) == MC_PARSE_OK && c.mode == MC_MODE_POSITION);
    CHECK(parse("mode state", &c) == MC_PARSE_OK && c.mode == MC_MODE_STATE);
    CHECK(parse("sp -12.5", &c) == MC_PARSE_OK && c.kind == MC_CMD_SETPOINT &&
          c.value[0] == -12.5f);
    CHECK(parse("sp 6.2832e0", &c) == MC_PARSE_OK && c.value[0] == 6.2832f);
    CHECK(parse("gains pos 30 0 1.5", &c) == MC_PARSE_OK && c.kind == MC_CMD_GAINS &&
          c.loop == MC_LOOP_POSITION && c.value[0] == 30.0f && c.value[1] == 0.0f &&
          c.value[2] == 1.5f);
    CHECK(parse("slew speed 200", &c) == MC_PARSE_OK && c.kind == MC_CMD_SLEW &&
          c.loop == MC_LOOP_SPEED && c.value[0] == 200.0f);
    CHECK(parse("tele 10", &c) == MC_PARSE_OK && c.kind == MC_CMD_TELEMETRY && c.divider == 10);
    CHECK(parse("tele 0", &c) == MC_PARSE_OK && c.divider == 0);
    CHECK(parse("status", &c) == MC_PARSE_OK && c.kind == MC_CMD_STATUS);
    CHECK(parse("help", &c) == MC_PARSE_OK && c.kind == MC_CMD_HELP);
}

static void test_tolerates_case_whitespace_and_comments(void)
{
    mc_cmd_t c;
    CHECK(parse("  MODE   Pos\r\n", &c) == MC_PARSE_OK && c.mode == MC_MODE_POSITION);
    CHECK(parse("\tsp 1.5 # one and a half rad", &c) == MC_PARSE_OK && c.value[0] == 1.5f);
    CHECK(parse("", &c) == MC_PARSE_EMPTY);
    CHECK(parse("   \r\n", &c) == MC_PARSE_EMPTY);
    CHECK(parse("# comment only", &c) == MC_PARSE_EMPTY);
}

static void test_reports_errors(void)
{
    mc_cmd_t c;
    char long_line[MC_CMD_MAX_LINE + 8];
    CHECK(parse("fly away", &c) == MC_PARSE_UNKNOWN);
    CHECK(parse("mode turbo", &c) == MC_PARSE_UNKNOWN);
    CHECK(parse("gains torque 1 2 3", &c) == MC_PARSE_UNKNOWN);
    CHECK(parse("mode", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("sp", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("sp 1 2", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("sp 1.5x", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("gains speed 1 2", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("tele -1", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("tele 1.5", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("status now", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("a b c d e f g", &c) == MC_PARSE_BAD_ARGS);
    CHECK(parse("sp inf", &c) == MC_PARSE_OUT_OF_RANGE);
    CHECK(parse("sp nan", &c) == MC_PARSE_OUT_OF_RANGE);
    CHECK(parse("gains speed 1 -2 0", &c) == MC_PARSE_OUT_OF_RANGE);
    CHECK(parse("slew pos -5", &c) == MC_PARSE_OUT_OF_RANGE);
    CHECK(parse("tele 100001", &c) == MC_PARSE_OUT_OF_RANGE);
    memset(long_line, 'x', sizeof long_line - 1);
    long_line[sizeof long_line - 1] = '\0';
    CHECK(parse(long_line, &c) == MC_PARSE_BAD_ARGS);
    CHECK(strcmp(mc_parse_result_str(MC_PARSE_UNKNOWN), "unknown command or keyword") == 0);
}

static void test_apply_drives_the_servo(void)
{
    const mc_servo_config_t cfg = servo_config_from_params();
    mc_servo_t s;
    mc_cmd_t c;
    CHECK(mc_servo_init(&s, &cfg, 0) == MC_OK);
    CHECK(parse("mode volt", &c) == MC_PARSE_OK && mc_cmd_apply(&c, &s) == MC_OK);
    CHECK(s.mode == MC_MODE_VOLTAGE);
    CHECK(parse("sp 2.5", &c) == MC_PARSE_OK && mc_cmd_apply(&c, &s) == MC_OK);
    CHECK(mc_servo_update(&s, 0) == 2.5f);
    CHECK(parse("gains speed 2 30 0", &c) == MC_PARSE_OK && mc_cmd_apply(&c, &s) == MC_OK);
    CHECK(s.speed_pid.cfg.kp == 2.0f && s.speed_pid.cfg.ki == 30.0f);
    /* the speed PI has no derivative filter, so a D gain is refused */
    CHECK(parse("gains speed 2 30 0.1", &c) == MC_PARSE_OK && mc_cmd_apply(&c, &s) == MC_EINVAL);
    CHECK(parse("slew pos 5", &c) == MC_PARSE_OK && mc_cmd_apply(&c, &s) == MC_OK);
    CHECK(s.pos_slew == 5.0f);
    CHECK(parse("status", &c) == MC_PARSE_OK && mc_cmd_apply(&c, &s) == MC_EINVAL);
}

int main(void)
{
    RUN_TEST(test_parses_every_command);
    RUN_TEST(test_tolerates_case_whitespace_and_comments);
    RUN_TEST(test_reports_errors);
    RUN_TEST(test_apply_drives_the_servo);
    return mc_test_report();
}
