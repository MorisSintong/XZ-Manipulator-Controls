#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdlib.h>
#endif

/* Linked as an object into every host executable so archive dead-stripping
 * cannot omit startup protection from tests that use plain assert(). */
__attribute__((constructor)) static void suppress_failure_dialogs(void)
{
#ifdef _WIN32
    (void)SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX |
                       SEM_NOOPENFILEERRORBOX);
    (void)_set_abort_behavior(0U, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
}
