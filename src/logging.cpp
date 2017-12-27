/// FicTrac http://rjdmoore.net/fictrac/
/// \file       logging.cpp
/// \brief      Basic wrapper for boost::log.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "logging.h"

#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>

using std::string;

///
/// Initialise logging and sinks
///
void logging::init(string log_fn)
{
    boost::log::core::get()->set_filter
    (
        // trace < debug < info < warning < error < fatal
        boost::log::trivial::severity >= boost::log::trivial::info
    );
    //TODO: Add timestamp to log messages.
    auto cout_sink = boost::log::add_console_log(std::cout);
    auto file_sink = boost::log::add_file_log(log_fn);
}

///
/// Set logging verbosity level.
///
void logging::setVerbosity(string log_level)
{
    boost::log::trivial::severity_level s = boost::log::trivial::info;
    if (log_level == "trace") {
        s = boost::log::trivial::trace;
    } else if (log_level == "info") {
        s = boost::log::trivial::info;
    } else if (log_level == "debug") {
        s = boost::log::trivial::debug;
    } else if (log_level == "warning") {
        s = boost::log::trivial::warning;
    } else if (log_level == "error") {
        s = boost::log::trivial::error;
    } else if (log_level == "fatal") {
        s = boost::log::trivial::fatal;
    } else {
        // default
        log_level = "info";
    }
    BOOST_LOG_TRIVIAL(info) << "Setting logging verbosity to: " << log_level;
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= s);
}
