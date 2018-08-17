/// FicTrac http://rjdmoore.net/fictrac/
/// \file       TermRecorder.h
/// \brief      Implementation of stdout recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

class TermRecorder : public RecorderInterface
{
public:
    TermRecorder();
    ~TermRecorder();

    /// Interface to be overridden by implementations.
    bool open(std::string ignore = "") { _open = true; return true; }
    bool write(std::string s);
    void close() { _open = false; };

private:
};
