/// FicTrac http://rjdmoore.net/fictrac/
/// \file       timing.h
/// \brief      Time-related utility functions.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include <string>
#include <ctime>
#include <chrono>	// high_resolution_clock 
#include <thread>	// this_thread::sleep_for

extern const std::chrono::high_resolution_clock::time_point _t0;
extern const std::chrono::system_clock::time_point _tExec;

///
/// Return seconds since program start.
///
static double elapsed_secs() {
	using namespace std::chrono;
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	return duration_cast<duration<double>>(t1 - _t0).count();
}

///
/// Return system timestamp (ms)
///
static double ts_ms() {
    using namespace std::chrono;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    return duration_cast<microseconds>(t1.time_since_epoch()).count() / 1000.;
}

///
/// Return formatted date/time string for program launch.
///
static std::string execTime()
{
    static std::string s;

    if (s.empty()) {
        std::time_t t = std::chrono::system_clock::to_time_t(_tExec);
        struct tm* timeinfo = localtime(&t);

        char tmps[16];
        sprintf(tmps, "%4d%02d%02d_%02d%02d%02d",
            timeinfo->tm_year + 1900,
            timeinfo->tm_mon,
            timeinfo->tm_mday,
            timeinfo->tm_hour,
            timeinfo->tm_min,
            timeinfo->tm_sec);

        s = std::string(tmps);
    }
    return s;
}

///
/// Return formatted date/time string.
///
static std::string dateTimeString()
{
    time_t     rawtime;
    struct tm* timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    char tmps[16];
    sprintf(tmps, "%4d%02d%02d_%02d%02d%02d",
        timeinfo->tm_year + 1900,
        timeinfo->tm_mon,
        timeinfo->tm_mday,
        timeinfo->tm_hour,
        timeinfo->tm_min,
        timeinfo->tm_sec);

    return std::string(tmps);
}

///
/// Return formatted date string.
///
static std::string dateString()
{
    time_t     rawtime;
    struct tm* timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    char tmps[16];
    sprintf(tmps, "%02d/%02d/%4d",
        timeinfo->tm_mday,
        timeinfo->tm_mon+1,
        timeinfo->tm_year + 1900);

    return std::string(tmps);
}

///
/// Sleep (ms)
///
static void sleep(long ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
