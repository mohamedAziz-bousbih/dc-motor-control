/*
 * Quadrature encoder on the ESP32 pulse counter (PCNT), x4 decoding.
 *
 * Both channels of one PCNT unit count: channel 0 the edges of A (direction
 * from B), channel 1 the edges of B (direction from A), so every edge of
 * either signal is one count. The 16-bit hardware count is extended to 32
 * bits in software by mc_enc_ext (see mc_encoder.h), which only needs the
 * counter to be read regularly - no overflow interrupt.
 */
#ifndef QUAD_ENCODER_H
#define QUAD_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

bool quad_encoder_init(void);

/* Extended position in counts. Call from one task only. */
int32_t quad_encoder_read(void);

#endif /* QUAD_ENCODER_H */
