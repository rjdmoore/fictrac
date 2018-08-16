/// FicTrac http://rjdmoore.net/fictrac/
/// \file       misc.cpp
/// \brief      Miscellaneous utility functions.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "misc.h"

#include "timing.h"

#define NOMINMAX    // stop min/max macros being defined in windows.h
#include <windows.h>

#include <cstdio>

///
/// Helper function to force getchar to take new key press.
///
int getchar_clean()
{
    double t1 = elapsed_secs();
    int ret;
    do {
        ret = std::getchar();
    } while ((elapsed_secs() - t1) < 0.1);
    return ret;
}

///
/// Set process to high priority.
///
bool SetProcessHighPriority()
{
    /// Sets process class to high priority level - care should be taken if FicTrac process uses all available CPU...
    /// See https://docs.microsoft.com/en-us/windows/desktop/procthread/scheduling-priorities
    return SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
}

///
/// Set process to high priority.
///
bool SetThreadVeryHighPriority()
{
    /// Sets thread priority to high.
    /// See https://docs.microsoft.com/en-us/windows/desktop/procthread/scheduling-priorities
    return SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
}

bool SetThreadHighPriority()
{
    /// Sets thread priority to high.
    /// See https://docs.microsoft.com/en-us/windows/desktop/procthread/scheduling-priorities
    return SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
}

bool SetThreadNormalPriority()
{
    /// Sets thread priority to normal.
    /// See https://docs.microsoft.com/en-us/windows/desktop/procthread/scheduling-priorities
    return SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
}
