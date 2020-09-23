/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder.cpp
/// \brief      Implementation of socket recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#ifdef __APPLE__ || __linux__
#include "SocketRecorder_linux.src"
#elif _WIN32
#include "SocketRecorder_win.src"
#endif
