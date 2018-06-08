/// FicTrac http://rjdmoore.net/fictrac/
/// \file       timing.cpp
/// \brief      Time-related utility functions.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "timing.h"

static const std::chrono::high_resolution_clock::time_point _t0 = std::chrono::high_resolution_clock::now();
