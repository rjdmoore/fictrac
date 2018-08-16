/// FicTrac http://rjdmoore.net/fictrac/
/// \file       configGui.cpp
/// \brief      Executable wrapper for interactive config GUI.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//TODO: log file input argument

#include "Logger.h"
#include "ConfigGui.h"
#include "misc.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <exception>

using cv::Mat;
using std::string;


int main(int argc, char *argv[])
{
	PRINT("///");
    PRINT("/// configGui:\tA GUI for configuring FicTrac.\n///\t\tThis program should be run once for each new input source (or if the camera is moved).\n///");
    PRINT("/// Usage:\tconfigGui CONFIG_FN [-v LOG_VERBOSITY]\n///");
    PRINT("/// \tCONFIG_FN\tPath to input/output config file.");
    PRINT("/// \tLOG_VERBOSITY\t[Optional] One of DBG, INF, WRN, ERR.");
    PRINT("///");
    PRINT("/// Build date: %s", __DATE__);
    PRINT("///\n");

    /// Parse args.
    string log_level = "info";
    string config_fn = "config.txt";
    for (int i = 1; i < argc; ++i) {
        if ((string(argv[i]) == "--verbosity") || (string(argv[i]) == "-v")) {
            if (++i < argc) {
                log_level = argv[i];
            } else {
                LOG_ERR("-v/--verbosity requires one argument (debug < info (default) < warn < error)!");
                PRINT("\n\nHit ENTER to exit..");
                getchar_clean();
                return -1;
            }
        } else {
            config_fn = argv[i];
        }
    }
    
    /// Set logging level.
    Logger::setVerbosity(log_level);
    
    /// Init config object, parse config file.
    ConfigGui cfg(config_fn);
    if (!cfg.is_open()) {
        LOG_ERR("Error! Unable to open specified config file (%s).", config_fn.c_str());
        PRINT("\n\nHit ENTER to exit..");
        getchar_clean();
        return -1;
    }
    
    /// Run configuration GUI.
    bool ret = cfg.run();

    PRINT("\n\nHit ENTER to exit..");
    getchar_clean();

    return ret ? 0 : 1;
}
