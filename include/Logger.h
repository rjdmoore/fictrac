/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Logger.h
/// \brief      Simple thread-safe logger.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "Recorder.h"

#include <mutex>
#include <memory>   // unique_ptr
#include <string>
#include <fstream>  // ofstream

#define LOG(fmt, ...) Logger::mprintf(Logger::INF, __FUNCTION__, fmt, __VA_ARGS__)
#define LOG_DBG(fmt, ...) Logger::mprintf(Logger::DBG, __FUNCTION__, fmt, __VA_ARGS__)
#define LOG_WRN(fmt, ...) Logger::mprintf(Logger::WRN, __FUNCTION__, fmt, __VA_ARGS__)
#define LOG_ERR(fmt, ...) Logger::mprintf(Logger::ERR, __FUNCTION__, fmt, __VA_ARGS__)
#define PRINT(fmt, ...) Logger::mprintf(Logger::PRT, __FUNCTION__, fmt, __VA_ARGS__)


class Logger
{
public:
    // logger verbosity level
    enum LogLevel { DBG, INF, WRN, ERR, PRT };
    const char * const LogLevelStrings[4] = { "DBG", "INF", "WRN", "ERR" };

    /// Delete the copy constructors we wish to block (public decs give better compiler error msgs)
    Logger(Logger const&) = delete;
    void operator=(Logger const&) = delete;

    /// Get/set verbosity
    static LogLevel& verbosity() {
        static LogLevel v = INF;   // default to INF
        return v;
    };

    /// Verbosity helper functions
    static void setVerbosity(LogLevel v) {
        verbosity() = v;
    }

    static void setVerbosity(std::string v);

    /// Thread-safe printf wrapper
    static void mprintf(LogLevel lvl, std::string func, std::string format, ...);

private:
    /// Hidden constructor to prevent direct instantiation
    Logger();
    ~Logger();

private:
    std::unique_ptr<Recorder> _log;
    std::unique_ptr<Recorder> _cout;
    std::mutex _pMutex;
};
