/// FicTrac http://rjdmoore.net/fictrac/
/// \file       configGui.cpp
/// \brief      Executable wrapper for interactive config GUI.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//TODO: log file input argument

#include "ConfigGui.h"
#include "logging.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <exception>

using cv::Mat;
using std::string;

int main(int argc, char *argv[])
{
    /// Init logging.
    logging::init();

    BOOST_LOG_TRIVIAL(info) << "configGui:\tA GUI for configuring FicTrac.\n\nThis program should be run once for each new input source (or if the camera is moved).\n";
    BOOST_LOG_TRIVIAL(info) << "Usage: " << argv[0] << " INPUT -c CONFIG_FN [-v LOG_VERBOSITY]\n";
    
    /// Parse args.
    string input_fn = "0";     // default to primary webcam
    string log_level = "info";
    string config_fn = "config.txt";
    for (int i = 1; i < argc; ++i) {
        if ((string(argv[i]) == "--verbosity") || (string(argv[i]) == "-v")) {
            if (++i < argc) {
                log_level = argv[i];
            } else {
                BOOST_LOG_TRIVIAL(error) << "-v/--verbosity requires one argument (debug < info (default) < warning < error)!";
                return -1;
            }
        } else if ((string(argv[i]) == "--config") || (string(argv[i]) == "-c")) {
            if (++i < argc) {
                config_fn = argv[i];
            } else {
                BOOST_LOG_TRIVIAL(error) << "-c/--config requires one argument (config file name)!";
                return -1;
            }
        } else {
            input_fn = argv[i];
        }
    }
    
    /// Set logging level.
    logging::setVerbosity(log_level);
    
    /// Load and parse config file.
    BOOST_LOG_TRIVIAL(info) << "Looking for config file: " << config_fn;
    ConfigGui cfg(config_fn);
    if (!cfg.is_open()) { return -1; }
    
    /// Load an image to use for annotation.
    Mat input_frame;
    try {
        // try loading as image file first
        BOOST_LOG_TRIVIAL(debug) << "Trying to load input " << input_fn << " as image...";
        input_frame = cv::imread(input_fn, CV_LOAD_IMAGE_GRAYSCALE);
        if (input_frame.empty()) { throw 0; }
    } catch(...) {
        try {
            // then try loading as camera id
            BOOST_LOG_TRIVIAL(debug) << "Trying to load input " << input_fn << " as camera id...";
            int id = stoi(input_fn);
            cv::VideoCapture cap(id);
            cap >> input_frame;
            if (input_frame.empty()) { throw 0; }
        } catch(...) {
            try {
                // then try loading as video file
                BOOST_LOG_TRIVIAL(debug) << "Trying to load input " << input_fn << " as video file...";
                cv::VideoCapture cap(input_fn);
                cap >> input_frame;
                if (input_frame.empty()) { throw 0; }
            } catch(...) {
                BOOST_LOG_TRIVIAL(error) << "Could not load input file (" << input_fn << ")!";
                return -1;
            }
        }
    }
    
    /// Run configuration GUI.
    cfg.setFrame(input_frame);
    return cfg.run() ? 0 : -1;
}
