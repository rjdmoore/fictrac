/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SerialRecorder.cpp
/// \brief      Implementation of serial recorder.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "SerialRecorder.h"

#include "Logger.h"

#include <boost/asio.hpp>
#include <boost/exception/diagnostic_information.hpp>

#include <iostream>

using namespace std;
using namespace boost;

///
///
///
SerialRecorder::SerialRecorder()
{
    _type = COM;
}

///
///
///
SerialRecorder::~SerialRecorder()
{
    closeRecord();
}

///
///
///
bool SerialRecorder::openRecord(std::string port_baud)
{
    // extract port no and baud
    size_t pos = port_baud.find_first_of('@');
    if (pos == string::npos) {
        LOG_ERR("Error! Malformed port:baud string.");
        return false;
    }
    _port_name = port_baud.substr(0, pos);
    int baud = stoi(port_baud.substr(pos + 1));

    LOG("Opening serial port %s with baud rate %d", _port_name.c_str(), baud);

    // open serial port
    try {
        asio::io_service io;
        _port = make_shared<asio::serial_port>(io);
        _port->open(_port_name);
        //_port->set_option(asio::serial_port_base::baud_rate(baud));
        _open = _port->is_open();
        if (!_open) { throw; }
    }
    catch (const boost::system::system_error &e) {
        LOG_ERR("Error! Could not open serial port %s @ baud rate %d. Error was %s", _port_name.c_str(), baud, boost::diagnostic_information(e).c_str());
        _open = false;
    }
    catch (const boost::exception &e) {
        LOG_ERR("Error! Could not open serial port %s @ baud rate %d. Error was %s", _port_name.c_str(), baud, boost::diagnostic_information(e).c_str());
        _open = false;
    }
    catch (...) {
        LOG_ERR("Error! Could not open serial port %s @ baud rate %d.", _port_name.c_str(), baud);
        _open = false;
    }
    return _open;
}

///
///
///
bool SerialRecorder::writeRecord(string s)
{
    if (_open) {
        try {
            _port->write_some(asio::buffer(s));
        }
        catch (const boost::exception &e) {
            LOG_ERR("Error writing to serial port (%s)! Error was %s", _port_name.c_str(), boost::diagnostic_information(e).c_str());
            return false;
        }
    }
    return _open;
}

///
///
///
void SerialRecorder::closeRecord()
{
    LOG("Closing serial port %s", _port_name.c_str());

    _open = false;
    _port->close();
}
