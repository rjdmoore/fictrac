/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder.h
/// \brief      Implementation of socket recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#ifdef __linux__ 
#include "SocketRecorder_linux.h"
#elif _WIN32
#include "SocketRecorder_winsocket.h"
#endif
