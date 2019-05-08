/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SerialRecorder.h
/// \brief      Implementation of serial recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <SDKDDKVer.h>
#include <boost/asio/serial_port.hpp> 

#include <string>
#include <memory>

class SerialRecorder : public RecorderInterface
{
public:
    SerialRecorder();
    ~SerialRecorder();

    /// Interface to be overridden by implementations.
    bool openRecord(std::string port_baud);
    bool writeRecord(std::string s);
    void closeRecord();

private:
    std::string _port_name;
    std::shared_ptr<boost::asio::serial_port> _port;
};
