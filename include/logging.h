/// FicTrac http://rjdmoore.net/fictrac/
/// \file       logging.h
/// \brief      Basic wrapper for boost::log.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <string>

namespace logging
{
    /// Initialise logging and sinks.
    void init(std::string log_fn = "fictrac.log");
    
    /// Set verbosity level.
    void setVerbosity(std::string log_level);
}
