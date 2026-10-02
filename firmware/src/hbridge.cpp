#include "hbridge.h"

#include <math.h>

#include "board.h"

static float s_v_supply = 1.0f;
static int s_direction = 0; /* last level written to INA/INB: +1, -1 or 0 */

static constexpr uint32_t kDutyFull = 1u << BRIDGE_PWM_BITS; /* LEDC: 2^bits = 100 % */

bool hbridge_init(float v_supply)
{
    if (!(v_supply > 0.0f)) {
        return false;
    }
    s_v_supply = v_supply;

    gpio_config_t dir = {};
    dir.pin_bit_mask = (1ULL << PIN_BRIDGE_INA) | (1ULL << PIN_BRIDGE_INB);
    dir.mode = GPIO_MODE_OUTPUT;
    dir.pull_up_en = GPIO_PULLUP_DISABLE;
    dir.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dir.intr_type = GPIO_INTR_DISABLE;

    gpio_config_t diag = dir;
    diag.pin_bit_mask = 1ULL << PIN_BRIDGE_DIAG;
    diag.mode = GPIO_MODE_INPUT;
    diag.pull_up_en = GPIO_PULLUP_ENABLE; /* backs up the pull-up on the carrier */

    if (gpio_config(&dir) != ESP_OK || gpio_config(&diag) != ESP_OK) {
        return false;
    }
    gpio_set_level(PIN_BRIDGE_INA, 0);
    gpio_set_level(PIN_BRIDGE_INB, 0);
    s_direction = 0;

    ledc_timer_config_t timer = {};
    timer.speed_mode = BRIDGE_LEDC_MODE;
    timer.duty_resolution = BRIDGE_PWM_BITS;
    timer.timer_num = BRIDGE_LEDC_TIMER;
    timer.freq_hz = BRIDGE_PWM_FREQ_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;

    ledc_channel_config_t channel = {};
    channel.gpio_num = PIN_BRIDGE_PWM;
    channel.speed_mode = BRIDGE_LEDC_MODE;
    channel.channel = BRIDGE_LEDC_CHANNEL;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = BRIDGE_LEDC_TIMER;
    channel.duty = 0;
    channel.hpoint = 0;
    return ledc_timer_config(&timer) == ESP_OK && ledc_channel_config(&channel) == ESP_OK;
}

void hbridge_set_voltage(float u)
{
    if (!isfinite(u)) {
        u = 0.0f; /* fminf(NaN, 1) would be 1: full voltage */
    }
    const float d = fminf(fabsf(u) / s_v_supply, 1.0f);
    const uint32_t duty = (uint32_t)lroundf(d * (float)kDutyFull);
    const int direction = duty == 0u ? 0 : (u > 0.0f ? 1 : -1);
    if (direction != s_direction) {
        /* The bridge prevents cross-conduction internally, so the direction
         * may change while PWM is running. A new duty value is latched by
         * LEDC at the next PWM period (50 us), i.e. well within one sample. */
        gpio_set_level(PIN_BRIDGE_INA, direction > 0 ? 1 : 0);
        gpio_set_level(PIN_BRIDGE_INB, direction < 0 ? 1 : 0);
        s_direction = direction;
    }
    ledc_set_duty(BRIDGE_LEDC_MODE, BRIDGE_LEDC_CHANNEL, duty);
    ledc_update_duty(BRIDGE_LEDC_MODE, BRIDGE_LEDC_CHANNEL);
}

bool hbridge_fault(void)
{
    return gpio_get_level(PIN_BRIDGE_DIAG) == 0;
}
