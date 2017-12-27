/// FicTrac http://rjdmoore.net/fictrac/
/// \file       configGui.cpp
/// \brief      Executable wrapper for interactive config GUI.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//TODO: log file input argument

#include "ConfigGui.h"
#include "logging.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

using cv::Mat;
using std::string;

int main(int argc, char *argv[])
{
    /// Init logging.
    logging::init();
    
    /// Parse args.
    string log_level = "info";
    string config_fn = "config.txt";
    for (int i = 1; i < argc; ++i) {
        if ((string(argv[i]) == "--help") || (string(argv[i]) == "-h")) {
            printf("\nconfigGui:\tA GUI for configuring FicTrac.\n\t\tThis program should be run once for each new input source (or if the camera is moved).\n");
            printf("\nUsage: %s CONFIG_FN [-v LOG_VERBOSITY]\n\n", argv[0]);
            return 0;
        } else if ((string(argv[i]) == "--verbosity") || (string(argv[i]) == "-v")) {
            if (i+1 < argc) {
                log_level = argv[++i];
            } else {
                BOOST_LOG_TRIVIAL(error) << "-v/--verbosity requires one argument (debug < info (default) < warning < error)!";
                return -1;
            }
        } else {
            config_fn = argv[i];
        }
    }
    BOOST_LOG_TRIVIAL(info) << "Looking for config file: " << config_fn;
    
    /// Set logging level.
    logging::setVerbosity(log_level);
    
    /// Load and parse config file.
    ConfigGui cfg(config_fn);
    
    /// Load an image to use for annotation.
    Mat input_frame = cv::imread("data/webcam.png", CV_LOAD_IMAGE_GRAYSCALE);
    
    /// Run configuration GUI.
    cfg.setFrame(input_frame);
    return cfg.run() ? 0 : -1;
}
