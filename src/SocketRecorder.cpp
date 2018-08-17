/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder.cpp
/// \brief      Implementation of socket recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "SocketRecorder.h"

#include <ws2tcpip.h>
#include <stdio.h>
#include <iostream> // cout/cerr

///
///
///
SocketRecorder::SocketRecorder()
    : _listenSocket(INVALID_SOCKET), _clientSocket(INVALID_SOCKET)
{
    _type = SOCK;
}

///
///
///
SocketRecorder::~SocketRecorder()
{
    close();
}

///
///
///
bool SocketRecorder::open(std::string port)
{
    // Initialize Winsock
    int iResult = WSAStartup(MAKEWORD(2, 2), &_wsaData);
    if (iResult != 0) {
        std::cerr << "Error! Failed to initialise WinSock library (err = " << iResult << ")!" << std::endl;
        return false;
    }

    struct addrinfo *result = nullptr, hints;

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the local address and port to be used by the server
    iResult = getaddrinfo(NULL, port.c_str(), &hints, &result);
    if (iResult != 0) {
        std::cerr << "Error! Failed to resolve local address (err = " << iResult << ")!" << std::endl;
        return false;
    }

    // Create a SOCKET for the server to listen for client connections
    _listenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (_listenSocket == INVALID_SOCKET) {
        //FIXME: include IP address in error msg.
        std::cerr << "Error! Could not create valid socket on port " << port  << " (err = " << WSAGetLastError() << ")!" << std::endl;
        freeaddrinfo(result);
        return false;
    }

    // Setup the TCP listening socket
    iResult = bind(_listenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        std::cerr << "Error! Failed to bind socket on port " << port << " (err = " << WSAGetLastError() << ")!" << std::endl;
        freeaddrinfo(result);
        return false;
    }

    // After successful bind(), we no longer need address info.
    freeaddrinfo(result);

    // Listen on our socket.
    if (listen(_listenSocket, SOMAXCONN) == SOCKET_ERROR) {
        std::cerr << "Error! Failed to listen to socket on port " << port << " (err = " << WSAGetLastError() << ")!" << std::endl;
        return false;
    }

    // Wait for client connection...
    //FIXME: include IP:port info in this message
    std::cout << "\nWaiting for client connection to socket: " << port << " ..." << std::endl;
    std::fflush(stdout);
    _clientSocket = accept(_listenSocket, NULL, NULL);
    if (_clientSocket == INVALID_SOCKET) {
        std::cerr << "Error! Failed to accept socket connection (err = " << WSAGetLastError() << ")!" << std::endl;
        return false;
    }

    return (_open = true);
}

///
///
///
bool SocketRecorder::write(std::string s)
{
    if (_open) {
        int iSendResult = send(_clientSocket, s.c_str(), s.size(), 0);
        if (iSendResult == SOCKET_ERROR) {
            std::cerr << "Error! Send failed (err = " << WSAGetLastError() << ")!" << std::endl;
            _open = false;  // should this be a terminal error?
        }
    }
    return _open;
}

///
///
///
void SocketRecorder::close()
{
    _open = false;
    closesocket(_clientSocket);
    closesocket(_listenSocket);
    WSACleanup();
}
