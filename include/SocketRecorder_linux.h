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
    bool open(std::string port);
    bool write(std::string s);
    void close();

private:
    int _listenSocket, _clientSocket;
};
