/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder.h
/// \brief      Implementation of socket recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <winsock2.h>   // WSADATA, SOCKET
#include <string>

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
    WSADATA _wsaData;
    SOCKET _listenSocket, _clientSocket;
};
