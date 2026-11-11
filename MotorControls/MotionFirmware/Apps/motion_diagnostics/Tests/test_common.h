/* Minimal assertion helpers shared by the host unit tests. */
#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <math.h>
#include <stdio.h>

static int g_checks;
static int g_failures;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
        }                                                                      \
    } while (0)

#define CHECK_MSG(cond, ...)                                                   \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("    FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond);        \
            printf(__VA_ARGS__);                                               \
            printf("\n");                                                      \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    CHECK_MSG(fabs((double)(a) - (double)(b)) <= (double)(tol),                 \
              "%s=%g %s=%g tol=%g", #a, (double)(a), #b, (double)(b), (double)(tol))

#define RUN_TEST(fn)                                                           \
    do {                                                                       \
        const int before_ = g_failures;                                        \
        printf("[ RUN  ] %s\n", #fn);                                          \
        fn();                                                                  \
        printf("%s %s\n", (g_failures == before_) ? "[  OK  ]" : "[ FAIL ]", #fn); \
    } while (0)

#define TEST_SUMMARY()                                                         \
    (printf("%d checks, %d failures\n", g_checks, g_failures), (g_failures != 0) ? 1 : 0)

#endif /* TEST_COMMON_H */
