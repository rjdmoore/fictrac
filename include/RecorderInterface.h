/// FicTrac http://rjdmoore.net/fictrac/
/// \file       RecorderInterface.h
/// \brief      Template for recording interfaces.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include <string>

class RecorderInterface
{
public:
    enum RecordType {
        CLOSED,
        TERM,
        FILE,
        SOCK
    };

    RecorderInterface() : _open(false), _type(CLOSED) {}
    ~RecorderInterface() { _open = false; _type = CLOSED; } // just to make sure

    /// Delete the copy constructors we wish to block (public decs give better compiler error msgs)
    RecorderInterface(RecorderInterface const&) = delete;
    void operator=(RecorderInterface const&) = delete;

    bool is_open() { return _open; }
    RecordType type() { return _type; }

    /// Interface to be overridden by implementations.
    virtual bool open(std::string f = "") = 0;
    virtual bool write(std::string s) = 0;
    virtual void close() = 0;

protected:
    bool _open;
    RecordType _type;
};
