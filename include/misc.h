/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Misc.h
/// \brief      Miscellaneous common functions.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#ifdef WIN32
#include <Windows.h>
#endif

void mySleep(long ms)
{
#ifdef WIN32
	Sleep(ms);
#endif
}
