/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SerialRecorder.cpp
/// \brief      Implementation of serial recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#ifdef __linux__ 
#include "SerialRecorder_linux.src"
#elif _WIN32
#include "SerialRecorder_win.src"
#endif
