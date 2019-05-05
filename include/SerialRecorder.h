/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SerialRecorder.h
/// \brief      Implementation of serial recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#ifdef __linux__ 
#include "SerialRecorder_linux.h"
#elif _WIN32
#include "SerialRecorder_win.h"
#endif
