/*
 * Minimal self-checking test helpers: every CHECK is counted, failures are
 * printed with file and line, and main() returns non-zero if any failed.
 */
#ifndef MC_TEST_H
#define MC_TEST_H

#include <math.h>
#include <stdio.h>

static int mc_test_checks;
static int mc_test_failures;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        ++mc_test_checks;                                                              \
        if (!(cond)) {                                                                 \
            ++mc_test_failures;                                                        \
            fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);   \
        }                                                                              \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                              \
    do {                                                                               \
        const double a_ = (double)(actual);                                            \
        const double e_ = (double)(expected);                                          \
        const double t_ = (double)(tol);                                               \
        ++mc_test_checks;                                                              \
        if (!(fabs(a_ - e_) <= t_)) {                                                  \
            ++mc_test_failures;                                                        \
            fprintf(stderr, "%s:%d: %s = %.9g, expected %.9g +- %.3g\n", __FILE__,     \
                    __LINE__, #actual, a_, e_, t_);                                    \
        }                                                                              \
    } while (0)

#define RUN_TEST(fn)                                                                   \
    do {                                                                               \
        const int before_ = mc_test_failures;                                          \
        fn();                                                                          \
        printf("%-44s %s\n", #fn, mc_test_failures == before_ ? "ok" : "FAILED");      \
    } while (0)

static inline int mc_test_report(void)
{
    printf("%d checks, %d failed\n", mc_test_checks, mc_test_failures);
    return mc_test_failures == 0 ? 0 : 1;
}

#endif /* MC_TEST_H */
