/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Logger.cpp
/// \brief      Simple thread-safe logger.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "Logger.h"

#include "timing.h"

#include <cstdio>   // vsnprintf
#include <cstdarg>  // va_list, va_start, va_end
#include <iostream> // cout

using namespace std;

Logger::Logger()
{
    // create log writer
    string fn = string("fictrac-") + execTime() + ".log";
    _log = unique_ptr<Recorder>(new Recorder(RecorderInterface::RecordType::FILE, fn));
    _cout = unique_ptr<Recorder>(new Recorder(RecorderInterface::RecordType::TERM));
    if (_log->is_active() && _cout->is_active()) {
        cout << "Initialised logging to " << fn << endl;
    } else {
        cerr << "Error opening log file (" << fn << ") or cout stream!" << endl;
    }
}

Logger::~Logger()
{
}

void Logger::setVerbosity(std::string v) {
    if ((v.compare("debug") == 0) || (v.compare("DBG") == 0) || (v.compare("dbg") == 0)) {
        setVerbosity(DBG);
    }
    else if ((v.compare("info") == 0) || (v.compare("INF") == 0) || (v.compare("inf") == 0)) {
        setVerbosity(INF);
    }
    else if ((v.compare("warn") == 0) || (v.compare("WRN") == 0) || (v.compare("wrn") == 0)) {
        setVerbosity(WRN);
    }
    else if ((v.compare("error") == 0) || (v.compare("ERR") == 0) || (v.compare("err") == 0)) {
        setVerbosity(ERR);
    }
    else {
        setVerbosity(INF);
        LOG_WRN("Warning, verbosity (%s) not recognised! Defaulting to INFO.", v.c_str());
    }
}

/// Thread-safe printf wrapper.
void Logger::mprintf(LogLevel lvl, string func, string format, ...)
{
    static Logger log;   // *the* logger instance

    static const int buf_size = 1024;
    static char buf[buf_size];

    // not re-entrant
    lock_guard<mutex> l1(log._pMutex);

    // print and log
    if ((int)lvl >= (int)verbosity()) {
        
        // expand args
        va_list args;
        va_start(args, format);
        vsnprintf(buf, buf_size, format.c_str(), args);
        va_end(args);

        // async printing to console
        log._cout->addMsg(string(buf) + "\n");

        // don't log display text to file
        if (lvl != PRT) {
            // async logging to file (with additional info)
            log._log->addMsg(to_string(elapsed_secs()) + " " + func + " [" + log.LogLevelStrings[lvl] + "] " + buf + "\n");
        }
    }
}
