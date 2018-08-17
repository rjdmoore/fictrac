/// FicTrac http://rjdmoore.net/fictrac/
/// \file       FileRecorder.cpp
/// \brief      Implementation of file recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "FileRecorder.h"

#include <iostream> // cout/cerr

///
///
///
FileRecorder::FileRecorder()
{
    _type = FILE;
}

///
///
///
FileRecorder::~FileRecorder()
{
    close();
}

///
///
///
bool FileRecorder::open(std::string fn)
{
    _file.open(fn);
    _open = _file.is_open();
    if (!_open) {
        std::cerr << "Error! FileRecorder could not open output file (" << fn << ")!" << std::endl;
    }
    return _open;
}

///
///
///
bool FileRecorder::write(std::string s)
{
    if (!_open) { return false; }
    _file << s;
    return true;
}

///
///
///
void FileRecorder::close()
{
    _open = false;
    _file.close();
}
