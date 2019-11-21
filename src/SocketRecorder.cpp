/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder.cpp
/// \brief      Implementation of socket recorder as a ZeroMQ publisher.
/// \author     David Turner
/// \copyright  CC BY-NC-SA 3.0

#include "SocketRecorder.h"

#include "zhelpers.hpp"

#include <string>
#include <iostream>
#include <exception>  // try, catch

///
///
///
SocketRecorder::SocketRecorder()
{
    _type = SOCK;
}

///
///
///
SocketRecorder::~SocketRecorder()
{
    closeRecord();
}

///
///
///
bool SocketRecorder::openRecord(std::string port)
{
	context = std::unique_ptr<zmq::context_t>(new zmq::context_t(1));
    publisher = std::unique_ptr<zmq::socket_t>(new zmq::socket_t(*context, ZMQ_PUB));
    std::string connection_string = std::string("tcp://*:") + port;
    publisher->bind(connection_string.c_str());
    return (_open = true);
}

///
///
///
bool SocketRecorder::writeRecord(std::string s)
{
	std::cout << "Writing to socket!\n";
    if (_open) 
    	s_send(*publisher, s);
    
    return _open;
}

///
///
///
void SocketRecorder::closeRecord()
{
    _open = false;
}