#include <assert.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdlib.h>
#endif

int main(void)
{
#ifdef _WIN32
    const UINT required = SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX |
                          SEM_NOOPENFILEERRORBOX;
    assert((GetErrorMode() & required) == required);
    const unsigned previous = _set_abort_behavior(0U, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    assert((previous & (_WRITE_ABORT_MSG | _CALL_REPORTFAULT)) == 0U);
#endif
    return 0;
}
