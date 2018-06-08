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

///
/// Return seconds since program start.
///
static double ts_secs() {
	using namespace std::chrono;
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	return duration_cast<duration<double>>(t1 - _t0).count();
}

///
/// Return formatted date/time string.
///
static std::string dateString() {
	time_t     rawtime;
	struct tm* timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	char tmps[16];
	sprintf(tmps, "%4d%02d%02d_%02d%02d%02d",
		timeinfo->tm_year+1900,
		timeinfo->tm_mon,
		timeinfo->tm_mday,
		timeinfo->tm_hour,
		timeinfo->tm_min,
		timeinfo->tm_sec);

	return std::string(tmps);
}

///
/// Sleep (ms)
///
static void sleep(long ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
