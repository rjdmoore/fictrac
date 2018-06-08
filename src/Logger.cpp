/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Logger.cpp
/// \brief      Simple threadsafe logger.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "Logger.h"

FILE * _outputfile;
bool _fileopened = false;
