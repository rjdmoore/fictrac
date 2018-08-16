/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Recorder.cpp
/// \brief      Simple threaded writer.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "Recorder.h"

#include "misc.h"

#include <iostream> // cout/cerr

using namespace std;

Recorder::Recorder(string fn)
    : _active(false), _out(nullptr), _isFile(false)
{
    /// Open source.
    std::streambuf* buf = nullptr;
    if (!fn.empty()) {
        _file.open(fn);
        if (_file.is_open()) {
            buf = _file.rdbuf();
            _isFile = true;
        }
        else {
            cerr << "Error! Recorder could not open output file (" << fn << ") and will pipe to cout instead!" << endl;
        }
    }
    if (!_isFile) {
        buf = cout.rdbuf();
        _isFile = false;
    }

    /// Set stream buffer source.
    _out.rdbuf(buf);

    /// Activate thread.
    _active = true;
    _thread = unique_ptr<thread>(new thread(&Recorder::processMsgQ, this));
}

Recorder::~Recorder()
{
    cout << "Closing recorder.." << endl;

    unique_lock<mutex> l(_qMutex);
    _active = false;
    _qCond.notify_all();
    l.unlock();

    if (_thread && _thread->joinable()) {
        _thread->join();
    }

    if (_isFile) {
        _file.close();
    }
}

bool Recorder::addMsg(string msg)
{
    bool ret = false;
    lock_guard<mutex> l(_qMutex);
    if (_active) {
        _msgQ.push_back(msg);
        _qCond.notify_all();
        ret = true;
    }
    return ret;
}

void Recorder::processMsgQ()
{
    /// Set thread high priority (when run as SU).
    if (!SetThreadNormalPriority()) {
        cerr << "Error! Recorder processing thread unable to set thread priority!" << endl;
    }

    /// Get a un/lockable lock.
    unique_lock<mutex> l(_qMutex);
    while (_active) {
        while (_msgQ.size() == 0) {
            _qCond.wait(l);
            if (!_active) { break; }
        }

        /// Process msg queue. Ignore _active while we have message still to process.
        while (_msgQ.size() > 0) {
            string msg = _msgQ.front();
            _msgQ.pop_front();
            l.unlock();

            // do async i/o
            _out << msg.c_str();
            l.lock();
        }
    }
    l.unlock();
}
