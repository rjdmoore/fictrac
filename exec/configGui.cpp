/// FicTrac http://rjdmoore.net/fictrac/
/// \file       configGui.cpp
/// \brief      Executable wrapper for interactive config GUI.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//TODO: log file input argument

#include "ConfigGui.h"

#include "Logger.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <exception>

using cv::Mat;
using std::string;

int main(int argc, char *argv[])
{
    Logger::setLogFile("something.txt");

	PRINT("///");
    PRINT("/// configGui:\tA GUI for configuring FicTrac.\n///\n/// This program should be run once for each new input source (or if the camera is moved).");
    PRINT("/// Usage: configGui INPUT -c CONFIG_FN [-v LOG_VERBOSITY]");
    PRINT("///\n");

    /// Parse args.
    string input_fn = "0";     // default to primary webcam
    string log_level = "info";
    string config_fn = "config.txt";
    for (int i = 1; i < argc; ++i) {
        if ((string(argv[i]) == "--verbosity") || (string(argv[i]) == "-v")) {
            if (++i < argc) {
                log_level = argv[i];
            } else {
                LOG_ERR("-v/--verbosity requires one argument (debug < info (default) < warning < error)!");
                return -1;
            }
        } else if ((string(argv[i]) == "--config") || (string(argv[i]) == "-c")) {
            if (++i < argc) {
                config_fn = argv[i];
            } else {
                LOG_ERR("-c/--config requires one argument (config file name)!");
                return -1;
            }
        } else {
            input_fn = argv[i];
        }
    }
    
    /// Set logging level.
    Logger::setVerbosity(log_level);
    
    /// Load and parse config file.
    LOG("Looking for config file: %s ...", config_fn.c_str());
    ConfigGui cfg(config_fn);
    if (!cfg.is_open()) { return -1; }
    
    /// Load an image to use for annotation.
    Mat input_frame;
    try {
        // try loading as image file first
        LOG_DBG("Trying to load input %s as image ...", input_fn);
        input_frame = cv::imread(input_fn, CV_LOAD_IMAGE_GRAYSCALE);
        if (input_frame.empty()) { throw 0; }
    } catch(...) {
        try {
            // then try loading as camera id
            LOG_DBG("Trying to load input %s as camera id ...", input_fn);
            int id = stoi(input_fn);
            cv::VideoCapture cap(id);
            cap >> input_frame;
            if (input_frame.empty()) { throw 0; }
        } catch(...) {
            try {
                // then try loading as video file
                LOG_DBG("Trying to load input %s as video file ...", input_fn);
                cv::VideoCapture cap(input_fn);
                cap >> input_frame;
                if (input_frame.empty()) { throw 0; }
            } catch(...) {
                LOG_ERR("Could not load input file (%s)!", input_fn);
                return -1;
            }
        }
    }
    
    /// Run configuration GUI.
    cfg.setFrame(input_frame);
    return cfg.run() ? 0 : -1;
}
