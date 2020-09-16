/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder.h
/// \brief      Implementation of socket recorder based on boost::asio UDP datagrams.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <boost/asio.hpp>

class SocketRecorder : public RecorderInterface
{
public:
    SocketRecorder();
    ~SocketRecorder();

    /// Interface to be overridden by implementations.
    bool openRecord(std::string host_port);
    bool writeRecord(std::string s);
    void closeRecord();

private:
    std::string _host;
    int _port;

    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _socket;
    boost::asio::ip::udp::endpoint _endpoint;
};
