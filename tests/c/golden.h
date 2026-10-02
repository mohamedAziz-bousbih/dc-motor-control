/*
 * Reader for the golden-vector files in tests/golden (written by
 * control/golden.py):
 *
 *   # comment
 *   #@ name value        parameter
 *   col_a,col_b,...      header
 *   1,2.5,...            data rows
 *
 * Values are kept as double; the generator rounded every input to float32
 * and printed 9 significant digits, so casting to float is exact.
 */
#ifndef GOLDEN_H
#define GOLDEN_H

#include <stdbool.h>
#include <stddef.h>

#define GOLDEN_MAX_PARAMS 64
#define GOLDEN_MAX_COLS 16
#define GOLDEN_NAME_LEN 24

typedef struct {
    char param_names[GOLDEN_MAX_PARAMS][GOLDEN_NAME_LEN];
    double param_values[GOLDEN_MAX_PARAMS];
    size_t n_params;
    char columns[GOLDEN_MAX_COLS][GOLDEN_NAME_LEN];
    size_t n_cols;
    double *data; /* n_rows x n_cols, row-major */
    size_t n_rows;
} golden_t;

/* file is relative to GOLDEN_DIR. Prints the reason and returns false on error. */
bool golden_load(const char *file, golden_t *g);
void golden_free(golden_t *g);

/* Parameter value; aborts the test program if it is missing (a test bug). */
double golden_param(const golden_t *g, const char *name);

/* Column index; aborts if missing. */
size_t golden_column(const golden_t *g, const char *name);

static inline double golden_at(const golden_t *g, size_t row, size_t col)
{
    return g->data[row * g->n_cols + col];
}

/* Tracks the worst deviation of a replayed signal from its reference. */
typedef struct {
    const char *name;
    double atol, rtol;
    double max_err;
    size_t worst_row;
    size_t violations;
} golden_cmp_t;

void golden_cmp_add(golden_cmp_t *c, size_t row, double actual, double expected);
void golden_cmp_print(const golden_cmp_t *c);

#endif /* GOLDEN_H */
