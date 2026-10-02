/*
 * Shared definitions of the motorctl library.
 *
 * The library is portable C (C99/C11, single-precision float, no dynamic
 * memory, no global state). Every module has a Python reference model in
 * control/discrete.py or control/servo.py; the golden-vector tests in
 * tests/c compare the two sample by sample.
 */
#ifndef MOTORCTL_MC_COMMON_H
#define MOTORCTL_MC_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MC_OK = 0,
    MC_EINVAL = -1 /* invalid argument or configuration */
} mc_status_t;

/* Saturation. Written with comparisons only, so infinite limits work and a
 * NaN input passes through unchanged (it is caught by the caller's checks,
 * not hidden by the clamp). */
static inline float mc_clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

#ifdef __cplusplus
}
#endif

#endif /* MOTORCTL_MC_COMMON_H */
