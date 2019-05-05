/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SerialRecorder_linux.h
/// \brief      Linux implementation of serial recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <cstring>

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
};
