/* Deliberately invalid operations, compiled only for instrumentation self-tests. */
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc != 3) return 2;
    if (strcmp(argv[1], "asan") == 0) {
        size_t index = (size_t)strtoul(argv[2], NULL, 10);
        volatile unsigned char *buffer = (unsigned char *)malloc(8u);
        if (buffer == NULL) return 3;
        buffer[index] = 42u;
        printf("%u\n", (unsigned)buffer[index]);
        free((void *)buffer);
    } else {
        volatile int value = atoi(argv[2]);
        printf("%d\n", value + 1);
    }
    return 0;
}
