/// FicTrac http://rjdmoore.net/fictrac/
/// \file       misc.h
/// \brief      Miscellaneous utility functions.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

///
/// Helper function to force getchar to take new key press.
///
int getchar_clean();

///
/// Set process priority.
///
bool SetProcessHighPriority();

///
/// Set thread priority.
///
bool SetThreadVeryHighPriority();
bool SetThreadHighPriority();
bool SetThreadNormalPriority();
