/// FicTrac http://rjdmoore.net/fictrac/
/// \file       misc.cpp
/// \brief      Miscellaneous utility functions.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "misc.h"

#include "timing.h"

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
