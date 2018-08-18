/// FicTrac http://rjdmoore.net/fictrac/
/// \file       FileRecorder.h
/// \brief      Implementation of file recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <fstream>  // ofstream

class FileRecorder : public RecorderInterface
{
public:
    FileRecorder();
    ~FileRecorder();

    /// Interface to be overridden by implementations.
    bool openRecord(std::string fn = "");
    bool writeRecord(std::string s);
    void closeRecord();

private:
    std::ofstream _file;
};
