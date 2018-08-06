/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Recorder.cpp
/// \brief      Simple threaded writer.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "Recorder.h"

#include <iostream> // cout/cerr

using namespace std;

Recorder::Recorder(string fn)
    : _active(false)
{
    _file.open(fn.c_str());
    if (_file.is_open()) {
        _active = true;
        _t1 = unique_ptr<thread>(new thread(&Recorder::processMsgQ, this));
    }
    else {
        cerr << "Error! Recorder could not open output file (" << fn << ")!" << endl;
    }
}

Recorder::~Recorder()
{
    cout << "Closing recorder.." << endl;

    unique_lock<mutex> l(_qMutex);
    _active = false;
    _qCond.notify_all();
    l.unlock();

    if (_t1 && _t1->joinable()) {
        _t1->join();
    }

    _file.close();
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
            _file << msg.c_str();
            l.lock();
        }
    }
    l.unlock();
}
