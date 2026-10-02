#include "quad_encoder.h"

#include "board.h"
#include "motorctl/mc_encoder.h"

static mc_enc_ext_t s_ext;

static bool config_channel(pcnt_channel_t channel, gpio_num_t pulse, gpio_num_t ctrl,
                           pcnt_count_mode_t rising, pcnt_count_mode_t falling)
{
    pcnt_config_t c = {};
    c.pulse_gpio_num = pulse;
    c.ctrl_gpio_num = ctrl;
    c.lctrl_mode = PCNT_MODE_REVERSE; /* control input low: count the other way */
    c.hctrl_mode = PCNT_MODE_KEEP;
    c.pos_mode = rising;
    c.neg_mode = falling;
    c.counter_h_lim = ENC_PCNT_LIMIT;
    c.counter_l_lim = -ENC_PCNT_LIMIT;
    c.unit = ENC_PCNT_UNIT;
    c.channel = channel;
    return pcnt_unit_config(&c) == ESP_OK;
}

bool quad_encoder_init(void)
{
    /* A leads B in the positive direction. Channel 0 sees A rise while B is
     * low (reverse mode) -> +1; channel 1 sees B rise while A is high -> +1. */
    if (!config_channel(PCNT_CHANNEL_0, PIN_ENC_A, PIN_ENC_B, PCNT_COUNT_DEC, PCNT_COUNT_INC) ||
        !config_channel(PCNT_CHANNEL_1, PIN_ENC_B, PIN_ENC_A, PCNT_COUNT_INC, PCNT_COUNT_DEC)) {
        return false;
    }
    if (pcnt_set_filter_value(ENC_PCNT_UNIT, ENC_PCNT_FILTER) != ESP_OK ||
        pcnt_filter_enable(ENC_PCNT_UNIT) != ESP_OK || pcnt_counter_pause(ENC_PCNT_UNIT) != ESP_OK ||
        pcnt_counter_clear(ENC_PCNT_UNIT) != ESP_OK || pcnt_counter_resume(ENC_PCNT_UNIT) != ESP_OK) {
        return false;
    }
    int16_t raw = 0;
    if (pcnt_get_counter_value(ENC_PCNT_UNIT, &raw) != ESP_OK) {
        return false;
    }
    return mc_enc_ext_init(&s_ext, ENC_PCNT_LIMIT, raw) == MC_OK;
}

int32_t quad_encoder_read(void)
{
    int16_t raw = 0;
    if (pcnt_get_counter_value(ENC_PCNT_UNIT, &raw) != ESP_OK) {
        return s_ext.position; /* only fails for an invalid unit number */
    }
    return mc_enc_ext_update(&s_ext, raw);
}
