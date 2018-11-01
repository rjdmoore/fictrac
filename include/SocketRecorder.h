/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder.h
/// \brief      Implementation of socket recorder.
/// \author     Richard Moore, David Turner
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <cstring>
#include <memory>
#include <zmq.hpp>

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

	// ZeroMQ Context we will use for this publisher
    std::unique_ptr<zmq::context_t> context;
   
    // The ZeroMQ socket we will use for publishing.
    std::unique_ptr<zmq::socket_t> publisher;
};
