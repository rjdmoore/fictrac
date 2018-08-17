/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Recorder.h
/// \brief      Simple threaded writer.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "RecorderInterface.h"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>   // unique_ptr
#include <deque>
#include <string>


class Recorder
{
public:
    Recorder(RecorderInterface::RecordType type, std::string fn = "");
    ~Recorder();

    bool is_active() { return _active; }
    RecorderInterface::RecordType type() { return _record->type(); }

    /// Add msg to msgQ for async writing.
    bool addMsg(std::string msg);

private:
    void processMsgQ();

private:
    std::atomic<bool> _active;

    std::unique_ptr<RecorderInterface> _record;

    std::unique_ptr<std::thread> _thread;
    std::deque<std::string> _msgQ;
    std::mutex _qMutex;
    std::condition_variable _qCond;
};
