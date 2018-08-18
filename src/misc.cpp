/// FicTrac http://rjdmoore.net/fictrac/
/// \file       misc.cpp
/// \brief      Miscellaneous utility functions.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "misc.h"

#include "timing.h"

#ifdef __linux__ 
// linux inludes
#elif _WIN32
#define NOMINMAX    // stop min/max macros being defined in windows.h
#include <windows.h>
#endif


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
#ifdef __linux__ 
    // linux
    return false;
#elif _WIN32
    /// Sets process class to high priority level - care should be taken if FicTrac process uses all available CPU...
    /// See https://docs.microsoft.com/en-us/windows/desktop/procthread/scheduling-priorities
    return SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#endif
}

///
/// Set process to high priority.
///
bool SetThreadVeryHighPriority()
{
#ifdef __linux__ 
    // linux
    return false;
#elif _WIN32
    /// Sets thread priority to highest.
    /// See https://docs.microsoft.com/en-us/windows/desktop/procthread/scheduling-priorities
    return SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif
}

bool SetThreadHighPriority()
{
#ifdef __linux__ 
    // linux
    return false;
#elif _WIN32
    /// Sets thread priority to high.
    /// See https://docs.microsoft.com/en-us/windows/desktop/procthread/scheduling-priorities
    return SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif
}

bool SetThreadNormalPriority()
{
#ifdef __linux__ 
    // linux
    return false;
#elif _WIN32
    /// Sets thread priority to normal.
    /// See https://docs.microsoft.com/en-us/windows/desktop/procthread/scheduling-priorities
    return SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
#endif
}
