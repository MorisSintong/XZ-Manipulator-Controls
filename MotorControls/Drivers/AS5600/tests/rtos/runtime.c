#include "test_support.h"
#include "mps2_device.h"

#include <stddef.h>
#include <string.h>

static uint32_t semihost(uint32_t operation, const void *argument)
{
    register uint32_t r0 __asm("r0") = operation;
    register const void *r1 __asm("r1") = argument;
    __asm volatile ("bkpt 0xab" : "+r"(r0) : "r"(r1) : "memory");
    return r0;
}

void test_puts(const char *text)
{
    (void)semihost(4U, text);
}

void test_put_u32(uint32_t value)
{
    char text[11];
    unsigned index = sizeof(text) - 1U;
    text[index] = '\0';
    do
    {
        text[--index] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    test_puts(&text[index]);
}

void test_exit(uint32_t code)
{
    const uint32_t arguments[2] = {0x20026U, code};
    __disable_irq();
    (void)semihost(0x20U, arguments);
    for (;;)
    {
        __WFI();
    }
}

void test_assert_failed(const char *expression, const char *file, unsigned line)
{
    __disable_irq();
    test_puts("FAIL: ");
    test_puts(file);
    test_puts(":");
    test_put_u32(line);
    test_puts(": ");
    test_puts(expression);
    test_puts("\nRESULT: FAIL\n");
    test_exit(1U);
}

void *memcpy(void *destination, const void *source, size_t length)
{
    unsigned char *d = destination;
    const unsigned char *s = source;
    for (size_t i = 0; i < length; ++i)
    {
        d[i] = s[i];
    }
    return destination;
}

void *memmove(void *destination, const void *source, size_t length)
{
    unsigned char *d = destination;
    const unsigned char *s = source;
    if ((uintptr_t)d < (uintptr_t)s)
    {
        return memcpy(d, s, length);
    }
    while (length != 0U)
    {
        --length;
        d[length] = s[length];
    }
    return destination;
}

void *memset(void *destination, int value, size_t length)
{
    unsigned char *d = destination;
    for (size_t i = 0; i < length; ++i)
    {
        d[i] = (unsigned char)value;
    }
    return destination;
}

int memcmp(const void *left, const void *right, size_t length)
{
    const unsigned char *a = left;
    const unsigned char *b = right;
    for (size_t i = 0; i < length; ++i)
    {
        if (a[i] != b[i])
        {
            return (int)a[i] - (int)b[i];
        }
    }
    return 0;
}

size_t strlen(const char *text)
{
    size_t length = 0U;
    while (text[length] != '\0')
    {
        ++length;
    }
    return length;
}

void __aeabi_memcpy(void *destination, const void *source, size_t length)
{
    (void)memcpy(destination, source, length);
}

void __aeabi_memcpy4(void *destination, const void *source, size_t length)
{
    (void)memcpy(destination, source, length);
}

void __aeabi_memcpy8(void *destination, const void *source, size_t length)
{
    (void)memcpy(destination, source, length);
}

void __aeabi_memclr(void *destination, size_t length)
{
    (void)memset(destination, 0, length);
}

void __aeabi_memclr4(void *destination, size_t length)
{
    (void)memset(destination, 0, length);
}

void __aeabi_memclr8(void *destination, size_t length)
{
    (void)memset(destination, 0, length);
}

/* AEABI needs both quotient and remainder; the assembly shim handles its ABI. */
uint64_t test_unsigned_divide(uint64_t numerator, uint64_t denominator,
                              uint64_t *remainder)
{
    uint64_t quotient = 0U;
    uint64_t rest = 0U;
    CHECK(denominator != 0U);
    for (unsigned bit = 0U; bit < 64U; ++bit)
    {
        const uint32_t carry = (uint32_t)(rest >> 63U);
        rest = (rest << 1U) | (numerator >> 63U);
        numerator <<= 1U;
        quotient <<= 1U;
        if ((carry != 0U) || (rest >= denominator))
        {
            rest -= denominator;
            quotient |= 1U;
        }
    }
    *remainder = rest;
    return quotient;
}
