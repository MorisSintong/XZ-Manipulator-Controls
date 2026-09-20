/* Deliberately defective executable, built ONLY for sanitizer self-tests.
 * Never link this fixture into the library or firmware.
 */
#include <stdint.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc != 2) {
        return 2;
    }
    if (strcmp(argv[1], "address") == 0) {
        volatile uint8_t *bytes = (volatile uint8_t *)malloc(4U);
        volatile size_t outside = 4U;
        if (bytes == NULL) {
            return 3;
        }
        bytes[outside] = 1U;
        free((void *)bytes);
        return 0;
    }
    if (strcmp(argv[1], "undefined") == 0) {
        volatile int32_t maximum = INT32_MAX;
        volatile int32_t increment = 1;
        volatile int32_t overflow = maximum + increment;
        (void)overflow;
        return 0;
    }
    return 2;
}
