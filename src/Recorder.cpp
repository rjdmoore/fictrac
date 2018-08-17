/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Recorder.cpp
/// \brief      Simple threaded writer.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "Recorder.h"

#include "TermRecorder.h"
#include "FileRecorder.h"
#include "SocketRecorder.h"
#include "misc.h"   // thread priority

#include <iostream> // cout/cerr

using namespace std;

Recorder::Recorder(RecorderInterface::RecordType type, string fn)
    : _active(false)
{
    /// Set record type.
    switch (type) {
    case RecorderInterface::RecordType::TERM:
        _record = unique_ptr<TermRecorder>(new TermRecorder());
        break;
    case RecorderInterface::RecordType::FILE:
        _record = unique_ptr<FileRecorder>(new FileRecorder());
        break;
    case RecorderInterface::RecordType::SOCK:
        _record = unique_ptr<SocketRecorder>(new SocketRecorder());
    default:
        break;
    }

    /// Open record and start async recording.
    if (_record && _record->open(fn)) {
        _active = true;
        _thread = unique_ptr<thread>(new thread(&Recorder::processMsgQ, this));
    }
    else {
        cerr << "Error initialising recorder!" << endl;
    }
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

    /// _record->close() called by unique_ptr dstr.
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
            _record->write(msg);
            l.lock();
        }
    }
    l.unlock();
}
