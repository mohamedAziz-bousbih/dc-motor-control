#include "golden.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef GOLDEN_DIR
#error "GOLDEN_DIR must be defined by the build system"
#endif

static void strip_eol(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) {
        s[--n] = '\0';
    }
}

static bool parse_param(char *line, golden_t *g)
{
    char name[GOLDEN_NAME_LEN];
    double value;
    /* "#@ name value"; strtod (behind %lf) also accepts inf and -inf */
    if (sscanf(line, "#@ %23s %lf", name, &value) != 2 || g->n_params == GOLDEN_MAX_PARAMS) {
        return false;
    }
    strcpy(g->param_names[g->n_params], name);
    g->param_values[g->n_params++] = value;
    return true;
}

static bool parse_header(char *line, golden_t *g)
{
    for (char *tok = strtok(line, ","); tok != NULL; tok = strtok(NULL, ",")) {
        if (g->n_cols == GOLDEN_MAX_COLS || strlen(tok) >= GOLDEN_NAME_LEN) {
            return false;
        }
        strcpy(g->columns[g->n_cols++], tok);
    }
    return g->n_cols > 0;
}

static bool parse_row(const char *line, golden_t *g, size_t *capacity)
{
    if (g->n_rows == *capacity) {
        *capacity = *capacity ? 2 * *capacity : 256;
        double *p = realloc(g->data, *capacity * g->n_cols * sizeof *p);
        if (p == NULL) {
            return false;
        }
        g->data = p;
    }
    double *row = g->data + g->n_rows * g->n_cols;
    const char *p = line;
    for (size_t i = 0; i < g->n_cols; ++i) {
        char *end = NULL;
        row[i] = strtod(p, &end);
        if (end == p || (i + 1 < g->n_cols ? *end != ',' : *end != '\0')) {
            return false;
        }
        p = end + 1;
    }
    ++g->n_rows;
    return true;
}

bool golden_load(const char *file, golden_t *g)
{
    char path[512];
    char line[512];
    size_t capacity = 0;
    size_t line_no = 0;
    memset(g, 0, sizeof *g);
    snprintf(path, sizeof path, "%s/%s", GOLDEN_DIR, file);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "cannot open %s\n", path);
        return false;
    }
    bool ok = true;
    while (ok && fgets(line, sizeof line, f) != NULL) {
        ++line_no;
        strip_eol(line);
        if (strncmp(line, "#@", 2) == 0) {
            ok = parse_param(line, g);
        } else if (line[0] == '#' || line[0] == '\0') {
            continue;
        } else if (g->n_cols == 0) {
            ok = parse_header(line, g);
        } else {
            ok = parse_row(line, g, &capacity);
        }
    }
    fclose(f);
    if (!ok || g->n_rows == 0) {
        fprintf(stderr, "%s:%zu: malformed golden file\n", path, line_no);
        golden_free(g);
        return false;
    }
    return true;
}

void golden_free(golden_t *g)
{
    free(g->data);
    g->data = NULL;
    g->n_rows = 0;
}

double golden_param(const golden_t *g, const char *name)
{
    for (size_t i = 0; i < g->n_params; ++i) {
        if (strcmp(g->param_names[i], name) == 0) {
            return g->param_values[i];
        }
    }
    fprintf(stderr, "golden file has no parameter '%s'\n", name);
    exit(2);
}

size_t golden_column(const golden_t *g, const char *name)
{
    for (size_t i = 0; i < g->n_cols; ++i) {
        if (strcmp(g->columns[i], name) == 0) {
            return i;
        }
    }
    fprintf(stderr, "golden file has no column '%s'\n", name);
    exit(2);
}

void golden_cmp_add(golden_cmp_t *c, size_t row, double actual, double expected)
{
    const double err = fabs(actual - expected);
    if (!(err <= c->atol + c->rtol * fabs(expected))) {
        if (c->violations == 0) {
            fprintf(stderr, "%s: row %zu: got %.9g, expected %.9g\n", c->name, row, actual,
                    expected);
        }
        ++c->violations;
    }
    if (err > c->max_err || isnan(err)) {
        c->max_err = err;
        c->worst_row = row;
    }
}

void golden_cmp_print(const golden_cmp_t *c)
{
    printf("  %-28s max |error| %.3g (row %zu), %zu outside tolerance\n", c->name, c->max_err,
           c->worst_row, c->violations);
}
