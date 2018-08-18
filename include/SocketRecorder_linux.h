/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder_linux.h
/// \brief      Linux implementation of socket recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <cstring>

class SocketRecorder : public RecorderInterface
{
public:
    SocketRecorder();
    ~SocketRecorder();

    /// Interface to be overridden by implementations.
    bool openRecord(std::string port);
    bool writeRecord(std::string s);
    void closeRecord();

private:
    int _listenSocket, _clientSocket;
};
