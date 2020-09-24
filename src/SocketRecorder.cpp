/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SocketRecorder.cpp
/// \brief      Implementation of socket recorder based on boost::asio UDP datagrams.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "SocketRecorder.h"

#include "Logger.h"

#include <boost/asio.hpp>

#include <string>

using namespace std;
using boost::asio::ip::udp;

///
///
///
SocketRecorder::SocketRecorder()
    : _socket(_io_service)
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
bool SocketRecorder::openRecord(std::string host_port)
{
    // extract host name and port
    size_t pos = host_port.find_first_of(':');
    if (pos == string::npos) {
        LOG_ERR("Error! Malformed host:port string.");
        return false;
    }
    _host = host_port.substr(0, pos);
    _port = stoi(host_port.substr(pos + 1));

    _endpoint = udp::endpoint(boost::asio::ip::address::from_string(_host), _port);

    LOG("Opening UDP connection to %s:%d", _host.c_str(), _port);

    // open socket
    try {
        _socket.open(udp::v4());

        _open = _socket.is_open();
        if (!_open) { throw; }
    }
    catch (const boost::system::system_error& e) {
        LOG_ERR("Error! Could not open UDP connection to %s:%d due to %s", _host.c_str(), _port, e.what());
        _open = false;
    }
    catch (...) {
        LOG_ERR("Error! Could not open UDP connection to %s:%d.", _host.c_str(), _port);
        _open = false;
    }
    return _open;
}

///
///
///
bool SocketRecorder::writeRecord(string s)
{
    if (_open) {
        try {
            _socket.send_to(boost::asio::buffer(s), _endpoint);
        }
        catch (const boost::system::system_error& e) {
            LOG_ERR("Error writing to socket (%s:%d)! Error was %s", _host.c_str(), _port, e.what());
            return false;
        }
    }
    return _open;
}

///
///
///
void SocketRecorder::closeRecord()
{
    LOG("Closing UDP connection...");

    _open = false;
    _socket.close();
}
