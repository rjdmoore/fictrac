/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder_winsocket.h
/// \brief      Windows implementation of socket recorder.
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
    bool openRecord(std::string port);
    bool writeRecord(std::string s);
    void closeRecord();

private:
    WSADATA _wsaData;
    SOCKET _listenSocket, _clientSocket;
};
