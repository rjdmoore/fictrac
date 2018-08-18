/// FicTrac http://rjdmoore.net/fictrac/
/// \file       TermRecorder.cpp
/// \brief      Implementation of stdout recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "TermRecorder.h"

#include <iostream> // cout/cerr

///
///
///
TermRecorder::TermRecorder()
{
    _type = TERM;
}

///
///
///
TermRecorder::~TermRecorder()
{
    closeRecord();
}

///
///
///
bool TermRecorder::writeRecord(std::string s)
{
    if (!_open) { return false; }
    std::cout << s;
    fflush(stdout);     // force flush in Linux
    return true;
}
