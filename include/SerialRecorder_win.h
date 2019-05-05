/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SerialRecorder_win.h
/// \brief      Windows implementation of serial recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <windows.h>
#include <string>

class SerialRecorder : public RecorderInterface
{
public:
    SerialRecorder();
    ~SerialRecorder();

    /// Interface to be overridden by implementations.
    bool openRecord(std::string port_baud);
    bool writeRecord(std::string s);
    void closeRecord();

private:
    HANDLE _commHandle;
};
