/*
 * ESP32 firmware for the DC servo.
 *
 * Two tasks, one owner of the controller state:
 *
 *   control task (core 0, high priority, 1 kHz via xTaskDelayUntil)
 *       read encoder -> apply queued commands -> mc_servo_update -> PWM
 *       -> telemetry sample into a queue, timing statistics
 *
 *   Arduino loop task (core 1, priority 1)
 *       serial line input -> mc_cmd_parse -> request queue -> wait for reply
 *       telemetry queue -> CSV lines on the serial port
 *
 * Only the control task touches the mc_servo_t after setup(), so no mutex is
 * needed and printing can never delay a control period. Commands take effect
 * at the start of the next sample, the same order the golden-vector test
 * (tests/golden/servo.csv) replays.
 *
 * Serial output convention: telemetry rows are plain CSV, every other line
 * starts with '#', so a logger can split the stream without parsing it.
 */
#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdio.h>

#include <atomic>

#include "controller_params.h"
#include "hbridge.h"
#include "motorctl/mc_cmd.h"
#include "motorctl/mc_servo.h"
#include "quad_encoder.h"
#include "servo_config.h"

namespace {

constexpr TickType_t kPeriodTicks = (TickType_t)(CTRL_TS_S * (float)configTICK_RATE_HZ + 0.5f);
static_assert(kPeriodTicks >= 1, "sample time shorter than one RTOS tick");
static_assert((float)kPeriodTicks / (float)configTICK_RATE_HZ - CTRL_TS_S < 1e-6f &&
                  CTRL_TS_S - (float)kPeriodTicks / (float)configTICK_RATE_HZ < 1e-6f,
              "sample time must be a whole number of RTOS ticks");

constexpr int64_t kPeriodUs = (int64_t)(CTRL_TS_S * 1e6f + 0.5f);
constexpr UBaseType_t kControlPriority = configMAX_PRIORITIES - 5; /* below IPC and esp_timer */
constexpr BaseType_t kControlCore = 0;  /* the Arduino loop and the UART run on core 1 */
constexpr unsigned long kBaud = 921600;

/* Requests carry a sequence number that the reply echoes, so a late reply
 * to a request that timed out is never taken as the answer to a newer one. */
struct Request {
    mc_cmd_t cmd;
    uint32_t seq;
};

struct Snapshot {
    mc_mode_t mode;
    float target, r_eff, theta, omega, omega_hat, u;
    mc_pid_config_t speed, pos;
    float speed_slew, pos_slew;
    bool fault;
    uint32_t telemetry_divider;
    uint32_t telemetry_drops;
    uint32_t nonfinite_trips;
    uint32_t overruns;
    uint32_t max_exec_us;
    uint32_t max_jitter_us;
};

struct Reply {
    uint32_t seq;
    mc_cmd_kind_t kind;
    mc_status_t status;
    bool refused_by_fault;
    Snapshot snap;
};

struct TelemetrySample {
    uint32_t k;
    uint8_t mode;
    float r, theta, omega, omega_hat, u;
};

/* Owned by the control task once it runs. */
mc_servo_t g_servo;
bool g_fault = false;
uint32_t g_tele_divider = 0;
uint32_t g_tele_drops = 0;
uint32_t g_overruns = 0;
uint32_t g_max_exec_us = 0;
uint32_t g_max_jitter_us = 0;

QueueHandle_t g_requests;
QueueHandle_t g_replies;
QueueHandle_t g_telemetry;
std::atomic<bool> g_fault_event{false};     /* control task -> loop task, one-shot notices */
std::atomic<bool> g_nonfinite_event{false};

Snapshot snapshot()
{
    Snapshot s;
    s.mode = g_servo.mode;
    s.target = g_servo.target;
    s.r_eff = g_servo.r_eff;
    s.theta = g_servo.theta;
    s.omega = g_servo.omega;
    s.omega_hat = mc_servo_omega_hat(&g_servo);
    s.u = g_servo.u;
    s.speed = g_servo.speed_pid.cfg;
    s.pos = g_servo.pos_pid.cfg;
    s.speed_slew = g_servo.speed_slew;
    s.pos_slew = g_servo.pos_slew;
    s.fault = g_fault;
    s.telemetry_divider = g_tele_divider;
    s.telemetry_drops = g_tele_drops;
    s.nonfinite_trips = g_servo.nonfinite_trips;
    s.overruns = g_overruns;
    s.max_exec_us = g_max_exec_us;
    s.max_jitter_us = g_max_jitter_us;
    return s;
}

Reply execute(const Request &req)
{
    const mc_cmd_t &cmd = req.cmd;
    Reply r = {};
    r.seq = req.seq;
    r.kind = cmd.kind;
    r.status = MC_OK;
    switch (cmd.kind) {
    case MC_CMD_TELEMETRY:
        g_tele_divider = cmd.divider;
        break;
    case MC_CMD_STATUS:
        break;
    case MC_CMD_MODE:
        if (g_fault) {
            /* Leaving the fault state needs an explicit "mode off" with the
             * bridge no longer reporting a fault. */
            if (cmd.mode != MC_MODE_OFF || hbridge_fault()) {
                r.status = MC_EINVAL;
                r.refused_by_fault = true;
                break;
            }
            g_fault = false;
        }
        r.status = mc_cmd_apply(&cmd, &g_servo);
        break;
    default:
        r.status = mc_cmd_apply(&cmd, &g_servo);
        break;
    }
    r.snap = snapshot();
    return r;
}

void control_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    int64_t prev_start = 0;
    uint32_t k = 0;
    uint32_t tele_count = 0;
    for (;;) {
        const BaseType_t on_time = xTaskDelayUntil(&last_wake, kPeriodTicks);
        const int64_t start = esp_timer_get_time();

        /* Sample first, so the measurement instant has a fixed offset from
         * the tick regardless of how many commands are pending. */
        const int32_t counts = quad_encoder_read();

        Request req;
        while (xQueueReceive(g_requests, &req, 0) == pdTRUE) {
            const Reply rep = execute(req);
            (void)xQueueSend(g_replies, &rep, 0);
        }
        if (!g_fault && hbridge_fault()) {
            g_fault = true;
            g_fault_event = true;
            (void)mc_servo_set_mode(&g_servo, MC_MODE_OFF);
        }

        const uint32_t trips_before = g_servo.nonfinite_trips;
        const float u = mc_servo_update(&g_servo, counts);
        if (g_servo.nonfinite_trips != trips_before) {
            g_nonfinite_event = true;
        }
        hbridge_set_voltage(u);

        if (g_tele_divider != 0 && ++tele_count >= g_tele_divider) {
            tele_count = 0;
            const TelemetrySample s = {k, (uint8_t)g_servo.mode, g_servo.r_eff, g_servo.theta,
                                       g_servo.omega, mc_servo_omega_hat(&g_servo), u};
            if (xQueueSend(g_telemetry, &s, 0) != pdTRUE) {
                ++g_tele_drops;
            }
        }

        if (on_time == pdFALSE) {
            ++g_overruns; /* the previous period ran past this wake-up time */
        }
        if (k > 0) {
            const int64_t dev = (start - prev_start) - kPeriodUs;
            const uint32_t jitter = (uint32_t)(dev < 0 ? -dev : dev);
            if (jitter > g_max_jitter_us) {
                g_max_jitter_us = jitter;
            }
        }
        const uint32_t exec = (uint32_t)(esp_timer_get_time() - start);
        if (exec > g_max_exec_us) {
            g_max_exec_us = exec;
        }
        prev_start = start;
        ++k;
    }
}

/* --- serial side (Arduino loop task) ------------------------------------- */

void print_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

void print_line(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) {
        Serial.write(reinterpret_cast<const uint8_t *>(buf),
                     (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1);
        Serial.write('\n');
    }
}

void print_help()
{
    static const char *const lines[] = {
        "# mode off|volt|speed|pos|state   switch controller (bumpless)",
        "# sp <value>                      setpoint: V, rad/s or rad, depending on mode",
        "# gains speed|pos <kp> <ki> <kd>  PID gains (bumpless); kd needs a filter (pos only)",
        "# slew speed|pos <rate>           setpoint rate limit [rad/s^2 | rad/s], 0 = off",
        "# tele <n>                        CSV telemetry every n-th sample, 0 = off",
        "# status                          mode, setpoint, gains, timing statistics",
        "# help                            this list",
    };
    for (const char *line : lines) {
        print_line("%s", line);
    }
}

void print_status(const Snapshot &s)
{
    print_line("# mode=%s target=%.5f r=%.5f theta=%.5f rad omega=%.4f omega_hat=%.4f rad/s u=%.3f V",
               mc_mode_name(s.mode), (double)s.target, (double)s.r_eff, (double)s.theta,
               (double)s.omega, (double)s.omega_hat, (double)s.u);
    print_line("# speed: kp=%.6g ki=%.6g kd=%.6g slew=%.6g", (double)s.speed.kp,
               (double)s.speed.ki, (double)s.speed.kd, (double)s.speed_slew);
    print_line("# pos:   kp=%.6g ki=%.6g kd=%.6g tf=%.6g slew=%.6g", (double)s.pos.kp,
               (double)s.pos.ki, (double)s.pos.kd, (double)s.pos.tf, (double)s.pos_slew);
    print_line("# fault=%d nonfinite_trips=%lu overruns=%lu max_exec=%lu us max_jitter=%lu us "
               "tele_div=%lu tele_drops=%lu",
               s.fault ? 1 : 0, (unsigned long)s.nonfinite_trips, (unsigned long)s.overruns,
               (unsigned long)s.max_exec_us, (unsigned long)s.max_jitter_us,
               (unsigned long)s.telemetry_divider, (unsigned long)s.telemetry_drops);
}

void handle_line(const char *line)
{
    mc_cmd_t cmd;
    const mc_parse_result_t parsed = mc_cmd_parse(line, &cmd);
    if (parsed == MC_PARSE_EMPTY) {
        return;
    }
    if (parsed != MC_PARSE_OK) {
        print_line("# error: %s", mc_parse_result_str(parsed));
        return;
    }
    if (cmd.kind == MC_CMD_HELP) {
        print_help();
        return;
    }
    static uint32_t s_seq = 0;
    const Request req = {cmd, ++s_seq};
    if (xQueueSend(g_requests, &req, pdMS_TO_TICKS(10)) != pdTRUE) {
        print_line("# error: command queue full");
        return;
    }
    /* Skip replies to earlier requests that timed out (they may still be in
     * the queue, or still be on their way if the control task was late). */
    const TickType_t timeout = pdMS_TO_TICKS(100);
    const TickType_t start = xTaskGetTickCount();
    Reply rep;
    for (;;) {
        const TickType_t waited = xTaskGetTickCount() - start; /* wrap-safe */
        if (waited >= timeout || xQueueReceive(g_replies, &rep, timeout - waited) != pdTRUE) {
            print_line("# error: no reply from the control task");
            return;
        }
        if (rep.seq == req.seq) {
            break;
        }
    }
    if (rep.refused_by_fault) {
        print_line("# error: bridge fault latched; clear it with 'mode off'");
    } else if (rep.status != MC_OK) {
        print_line("# error: rejected by the controller");
    } else if (rep.kind == MC_CMD_STATUS) {
        print_status(rep.snap);
    } else {
        print_line("# ok");
        if (rep.kind == MC_CMD_TELEMETRY && rep.snap.telemetry_divider != 0) {
            print_line("k,mode,r,theta,omega,omega_hat,u");
        }
    }
}

void print_telemetry(const TelemetrySample &s)
{
    print_line("%lu,%u,%.5f,%.5f,%.4f,%.4f,%.3f", (unsigned long)s.k, (unsigned)s.mode,
               (double)s.r, (double)s.theta, (double)s.omega, (double)s.omega_hat, (double)s.u);
}

char g_line[MC_CMD_MAX_LINE + 1];
size_t g_line_len = 0;
bool g_line_overflow = false;

void poll_serial()
{
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_line_overflow) {
                print_line("# error: line longer than %u characters", (unsigned)MC_CMD_MAX_LINE);
            } else {
                g_line[g_line_len] = '\0';
                handle_line(g_line);
            }
            g_line_len = 0;
            g_line_overflow = false;
        } else if (g_line_len < MC_CMD_MAX_LINE) {
            g_line[g_line_len++] = (char)c;
        } else {
            g_line_overflow = true;
        }
    }
}

[[noreturn]] void halt(const char *why)
{
    print_line("# init failed: %s", why);
    for (;;) {
        delay(1000);
    }
}

} // namespace

void setup()
{
    Serial.begin(kBaud);
    const mc_servo_config_t cfg = servo_config_from_params();
    if (!hbridge_init(cfg.v_max)) {
        halt("H-bridge (LEDC/GPIO)");
    }
    if (!quad_encoder_init()) {
        halt("encoder (PCNT)");
    }
    if (mc_servo_init(&g_servo, &cfg, quad_encoder_read()) != MC_OK) {
        halt("controller configuration");
    }
    g_requests = xQueueCreate(4, sizeof(Request));
    g_replies = xQueueCreate(4, sizeof(Reply));
    g_telemetry = xQueueCreate(128, sizeof(TelemetrySample));
    if (g_requests == nullptr || g_replies == nullptr || g_telemetry == nullptr) {
        halt("queues");
    }
    /* Pinned: ESP-IDF would pin an FPU-using task to its first core anyway,
     * and core 0 keeps it away from the Arduino loop and the UART driver. */
    if (xTaskCreatePinnedToCore(control_task, "control", 4096, nullptr, kControlPriority, nullptr,
                                kControlCore) != pdPASS) {
        halt("control task");
    }
    print_line("# dc-motor-control: Ts=%.4g s, %d counts/rev, mode off. 'help' lists the commands.",
               (double)CTRL_TS_S, CTRL_COUNTS_PER_REV);
}

void loop()
{
    TelemetrySample s;
    /* Blocks for at most 2 ms, so serial input is still polled when no
     * telemetry is running. */
    if (xQueueReceive(g_telemetry, &s, pdMS_TO_TICKS(2)) == pdTRUE) {
        print_telemetry(s);
        while (xQueueReceive(g_telemetry, &s, 0) == pdTRUE) {
            print_telemetry(s);
        }
    }
    if (g_fault_event.exchange(false)) {
        print_line("# fault: H-bridge DIAG low, output switched off");
    }
    if (g_nonfinite_event.exchange(false)) {
        print_line("# fault: controller output not finite, switched to mode off");
    }
    poll_serial();
}
