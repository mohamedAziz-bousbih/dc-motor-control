/*
 * Extension of a short hardware quadrature counter to 32 bits.
 *
 * The ESP32 PCNT counter is 16 bit and is reset to zero by the hardware when
 * it reaches its configured high or low limit (+-limit). Two successive
 * readings are therefore congruent modulo `limit` to the true positions. As
 * long as the counter is read at least once per limit/2 counts of motion,
 * the change between two readings is the unique representative of
 * (raw - last) mod limit in [-limit/2, limit/2), which this module adds to a
 * 32-bit position. No interrupt handler is needed, so there is no race
 * between an overflow ISR and the control task reading the counter.
 *
 * tests/c/test_encoder.c checks this against a simulated counter with resets.
 */
#ifndef MOTORCTL_MC_ENCODER_H
#define MOTORCTL_MC_ENCODER_H

#include <stdint.h>

#include "motorctl/mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t position; /* extended count */
    int16_t last_raw;
    int16_t limit;    /* counter resets to 0 at +limit and -limit */
} mc_enc_ext_t;

/* limit must be >= 2. position starts at 0 for the current raw reading. */
mc_status_t mc_enc_ext_init(mc_enc_ext_t *e, int16_t limit, int16_t raw0);

/* Feeds a new raw counter reading and returns the extended position. */
int32_t mc_enc_ext_update(mc_enc_ext_t *e, int16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* MOTORCTL_MC_ENCODER_H */
