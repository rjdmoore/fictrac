/// FicTrac http://rjdmoore.net/fictrac/
/// \file       configGui.cpp
/// \brief      Executable wrapper for interactive config GUI.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//TODO: log file input argument

#include "Logger.h"
#include "ConfigGui.h"
#include "misc.h"
#include "fictrac_version.h"

/// OpenCV individual includes required by gcc?
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <string>
#include <exception>

using cv::Mat;
using std::string;


int main(int argc, char *argv[])
{
	PRINT("///");
    PRINT("/// configGui:\tA GUI for configuring FicTrac.\n///\t\tThis program must be run for each new input source,\n///\t\tor if the camera is moved.\n///");
    PRINT("/// Usage:\tconfigGui CONFIG_FN [-v LOG_VERBOSITY -s SRC_FN]\n///");
    PRINT("/// \tCONFIG_FN\tPath to input/output config file.");
    PRINT("/// \tLOG_VERBOSITY\t[Optional] One of DBG, INF, WRN, ERR.");
    PRINT("/// \tSRC_FN\t\t[Optional] Override src_fn param in config file.");
    PRINT("///");
	PRINT("/// Version: %d.%d.%d (build date: %s)", FICTRAC_VERSION_MAJOR, FICTRAC_VERSION_MIDDLE, FICTRAC_VERSION_MINOR, __DATE__);
    PRINT("///\n");

    /// Parse args.
    string log_level = "info";
    string config_fn = "config.txt";
    string src_fn = "";
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
        } else if ((string(argv[i]) == "--src") || (string(argv[i]) == "-s")) {
            if (++i < argc) {
				src_fn = argv[i];
			}
			else {
                LOG_ERR("-s/--src requires one argument!");
				return -1;
			}
        } else {
            config_fn = argv[i];
        }
    }
    
    /// Set logging level.
    Logger::setVerbosity(log_level);
    
    /// Init config object, parse config file.
    ConfigGui cfg(config_fn, src_fn);
    if (!cfg.is_open()) {
        LOG_ERR("Error loading parameters from specified config file (%s)!", config_fn.c_str());
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
