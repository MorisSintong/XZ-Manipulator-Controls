#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <stdint.h>

void test_puts(const char *text);
void test_put_u32(uint32_t value);
void test_exit(uint32_t code) __attribute__((noreturn));
void test_assert_failed(const char *expression, const char *file, unsigned line)
    __attribute__((noreturn));

#define CHECK(expression) \
    do { if (!(expression)) { \
        test_assert_failed(#expression, __FILE__, __LINE__); \
    } } while (0)

#endif
