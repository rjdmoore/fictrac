/// FicTrac http://rjdmoore.net/fictrac/
/// \file       fictrac.cpp
/// \brief      FicTrac program.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "Logger.h"
#include "Trackball.h"
#include "timing.h"
#include "misc.h"
#include "fictrac_version.h"

#include <string>

using std::string;


int main(int argc, char *argv[])
{
     PRINT("///");
     PRINT("/// FicTrac:\tA webcam-based method for generating fictive paths.\n///");
     PRINT("/// Usage:\tfictrac CONFIG_FN [-v LOG_VERBOSITY]\n///");
     PRINT("/// \tCONFIG_FN\tPath to input config file (defaults to config.txt).");
     PRINT("/// \tLOG_VERBOSITY\t[Optional] One of DBG, INF, WRN, ERR.");
     PRINT("///");
     PRINT("/// Version: %2d.%02d (build date: %s)", FICTRAC_VERSION_MAJOR, FICTRAC_VERSION_MINOR, __DATE__);
     PRINT("///\n");

	/// Parse args.
	string log_level = "info";
	string config_fn = "config.txt";
	for (int i = 1; i < argc; ++i) {
		if ((string(argv[i]) == "--verbosity") || (string(argv[i]) == "-v")) {
			if (++i < argc) {
				log_level = argv[i];
			}
			else {
                LOG_ERR("-v/--verbosity requires one argument (debug < info (default) < warn < error)!");
				return -1;
			}
		}
		else {
            config_fn = argv[i];
		}
	}

    /// Set logging level.
    Logger::setVerbosity(log_level);

	//// Catch cntl-c
	//signal(SIGINT, TERMINATE);

	/// Set high priority (when run as SU).
    if (!SetProcessHighPriority()) {
        LOG_ERR("Error! Unable to set process priority!");
    } else {
        LOG("Set process priority to HIGH!");
    }

    Trackball tracker(config_fn);

    /// Now Trackball has spawned our worker threads, we set this thread to low priority.
    SetThreadNormalPriority();

    // wait for tracking to finish
    while (tracker.isActive()) { sleep(500); }

    //tracker.printState();
    tracker.writeTemplate();

    PRINT("\n\nHit ENTER to exit..");
    getchar_clean();
}
