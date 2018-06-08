/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Logger.h
/// \brief      Simple threadsafe logger.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "timing.h"		// dateString

#include <mutex>
#include <thread>
#include <cstdio>       // vprintf
#include <cstdarg>
#include <string>

/// Helper macro
#define MSG_FMT(fmt) std::to_string(ts_secs()) + "::" + std::string(__FUNCTION__) + ": " + (fmt) + "\n"

/// Logging macros
#define LOG(fmt, ...) Logger::mprintf(Logger::INF, MSG_FMT(fmt), __VA_ARGS__)
#define LOG_DBG(fmt, ...) Logger::mprintf(Logger::DBG, MSG_FMT(fmt), __VA_ARGS__)
#define LOG_WRN(fmt, ...) Logger::mprintf(Logger::WRN, MSG_FMT(fmt), __VA_ARGS__)
#define LOG_ERR(fmt, ...) Logger::mprintf(Logger::ERR, MSG_FMT(fmt), __VA_ARGS__)
#define DISP(fmt, ...) Logger::mprintf(Logger::DSP, std::string(fmt) + "\n", __VA_ARGS__)

#define SET_LOG_LEVEL(v) Logger::verbosity() = (v)

extern FILE * _outputfile;
extern bool _fileopened;


class Logger
{
public:
    ///
	/// Logger verbosity level (DBG < INF < WRN < ERR < DSP)
	///
    enum LOG_LEVEL { DBG, INF, WRN, ERR, DSP };

	///
    /// Get/set verbosity
	///
    static LOG_LEVEL& verbosity() {
        static LOG_LEVEL v = INF;   // default to INF
        return v;
    };

	///
	/// Copy stdout to logfile
	///
	static void logToFile(std::string fn = "") {
		if (fn.empty()) {
			fn = "logfile_" + dateString() + ".log";
		}
		_outputfile = fopen(fn.c_str(), "wt");
		_fileopened = true;
	}

	///
	/// Flush buffer
	///
	static void flush() {
		fflush(stdout);
		if (_fileopened) {
			fflush(_outputfile);
		}
	}

	///
	/// Printf wrapper - don't call directly; use macros!
	///
	static void mprintf(LOG_LEVEL lvl, std::string format, ...)
	{
		// re-entrant
		static std::mutex printf_mutex;
		std::lock_guard<std::mutex> lock(printf_mutex);

		if ((int)lvl >= (int)verbosity()) {
			va_list args;
			va_start(args, format);
			vprintf(format.c_str(), args);
            
            // also log to file
			if (_fileopened) {
				vfprintf(_outputfile, format.c_str(), args);
			}
			va_end(args);
		}
	};
};
