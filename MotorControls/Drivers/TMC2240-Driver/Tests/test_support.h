#ifndef TMC2240_TEST_SUPPORT_H
#define TMC2240_TEST_SUPPORT_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

extern uint64_t test_assertions;
extern uint32_t test_seed;
void test_failure(const char *expression, const char *file, int line);
uint32_t test_random(void);
void test_summary(const char *name, uint32_t properties, uint32_t recovery);
#define CHECK(x) do { ++test_assertions; if (!(x)) test_failure(#x, __FILE__, __LINE__); } while (0)
#define OK(x) CHECK((x) == TMC2240_OK)
#define STATUS(x, expected) CHECK((x) == (expected))
#endif
