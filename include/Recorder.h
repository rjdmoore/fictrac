/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Recorder.h
/// \brief      Simple threaded writer.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>   // unique_ptr
#include <deque>
#include <string>
#include <fstream>  // ofstream


class Recorder
{
public:
    Recorder(std::string fn = "");
    ~Recorder();

    bool is_active() { return _active; }

    /// Add msg to msgQ for async writing.
    bool addMsg(std::string msg);

private:
    void processMsgQ();

private:
    std::atomic<bool> _active;

    std::ostream _out;
    std::ofstream _file;
    bool _isFile;

    std::unique_ptr<std::thread> _thread;
    std::deque<std::string> _msgQ;
    std::mutex _qMutex;
    std::condition_variable _qCond;
};
