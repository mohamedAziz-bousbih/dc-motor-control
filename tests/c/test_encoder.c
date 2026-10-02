#include <stdint.h>

#include "mc_test.h"
#include "motorctl/mc_encoder.h"

/* Model of the ESP32 PCNT counter: counts one edge at a time and is reset to
 * zero by the hardware when it reaches +limit or -limit. */
typedef struct {
    int32_t value;
    int32_t limit;
} pcnt_model_t;

static void pcnt_count(pcnt_model_t *p, int32_t edges)
{
    const int32_t dir = edges >= 0 ? 1 : -1;
    for (int32_t i = 0; i != edges; i += dir) {
        p->value += dir;
        if (p->value == p->limit || p->value == -p->limit) {
            p->value = 0;
        }
    }
}

/* Deterministic pseudo-random motion (LCG), so failures are reproducible. */
static uint32_t lcg_state = 12345u;

static int32_t random_step(int32_t max_abs)
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (int32_t)((lcg_state >> 8) % (uint32_t)(2 * max_abs + 1)) - max_abs;
}

static void run_random_walk(int16_t limit, int32_t max_step, int32_t drift, int samples)
{
    pcnt_model_t hw = {0, limit};
    mc_enc_ext_t ext;
    CHECK(mc_enc_ext_init(&ext, limit, 0) == MC_OK);
    int32_t truth = 0;
    int mismatches = 0;
    for (int k = 0; k < samples; ++k) {
        const int32_t step = random_step(max_step) + drift;
        pcnt_count(&hw, step);
        truth += step;
        if (mc_enc_ext_update(&ext, (int16_t)hw.value) != truth) {
            ++mismatches;
        }
    }
    CHECK(mismatches == 0);
    /* the walk must actually have crossed the limit several times */
    CHECK(truth > 3 * limit || truth < -3 * limit);
}

static void test_tracks_position_across_many_resets(void)
{
    run_random_walk(100, 40, 7, 5000);     /* forward, |step| up to 47 < limit/2 */
    run_random_walk(100, 40, -7, 5000);    /* backward */
    run_random_walk(30000, 200, 25, 20000); /* firmware limit, fast motion */
    run_random_walk(31, 10, 2, 3000);       /* odd limit */
}

static void test_single_steps_across_the_reset_point(void)
{
    mc_enc_ext_t e;
    CHECK(mc_enc_ext_init(&e, 1000, 998) == MC_OK);
    CHECK(mc_enc_ext_update(&e, 999) == 1);
    CHECK(mc_enc_ext_update(&e, 0) == 2);    /* 999 -> 1000 = reset to 0 */
    CHECK(mc_enc_ext_update(&e, 3) == 5);
    CHECK(mc_enc_ext_update(&e, -2) == 0);   /* 3 -> 0 -> -2: no reset when passing 0 */

    CHECK(mc_enc_ext_init(&e, 1000, -998) == MC_OK);
    CHECK(mc_enc_ext_update(&e, -999) == -1);
    CHECK(mc_enc_ext_update(&e, 0) == -2);   /* -999 -> -1000 = reset to 0 */
    CHECK(mc_enc_ext_update(&e, -1) == -3);
    CHECK(mc_enc_ext_update(&e, 2) == 0);
}

static void test_half_limit_steps_resolve_to_the_documented_range(void)
{
    /* The change between two readings is taken from [-limit/2, limit/2):
     * exactly -limit/2 is a backward move, +limit/2 is the same residue and
     * is therefore also read as -limit/2. */
    mc_enc_ext_t e;
    CHECK(mc_enc_ext_init(&e, 100, 0) == MC_OK);
    CHECK(mc_enc_ext_update(&e, -50) == -50);
    CHECK(mc_enc_ext_init(&e, 100, 0) == MC_OK);
    CHECK(mc_enc_ext_update(&e, 49) == 49);
    CHECK(mc_enc_ext_init(&e, 100, 0) == MC_OK);
    CHECK(mc_enc_ext_update(&e, 50) == -50);
    /* odd limit: the range is [-15, 15] for limit 31 */
    CHECK(mc_enc_ext_init(&e, 31, 0) == MC_OK);
    CHECK(mc_enc_ext_update(&e, -15) == -15);
    CHECK(mc_enc_ext_init(&e, 31, 0) == MC_OK);
    CHECK(mc_enc_ext_update(&e, 15) == 15);
    CHECK(mc_enc_ext_init(&e, 31, 0) == MC_OK);
    CHECK(mc_enc_ext_update(&e, 16) == -15);
}

static void test_rejects_invalid_arguments(void)
{
    mc_enc_ext_t e;
    CHECK(mc_enc_ext_init(&e, 1, 0) == MC_EINVAL);
    CHECK(mc_enc_ext_init(&e, 100, 100) == MC_EINVAL);
    CHECK(mc_enc_ext_init(&e, 100, -100) == MC_EINVAL);
    CHECK(mc_enc_ext_init(NULL, 100, 0) == MC_EINVAL);
}

int main(void)
{
    RUN_TEST(test_tracks_position_across_many_resets);
    RUN_TEST(test_single_steps_across_the_reset_point);
    RUN_TEST(test_half_limit_steps_resolve_to_the_documented_range);
    RUN_TEST(test_rejects_invalid_arguments);
    return mc_test_report();
}
