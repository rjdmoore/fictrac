/// FicTrac http://rjdmoore.net/fictrac/
/// \file       ConfigGui.h
/// \brief      Interactive GUI for configuring FicTrac.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "ConfigParser.h"

#include "SharedPointers.h"
SHARED_PTR(CameraModel);

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

class ConfigGui
{
public:
    ConfigGui(std::string config_fn);
    ~ConfigGui();
    
    bool setFrame(cv::Mat& frame);
    
    bool run();
    
    bool is_open() { return _open; }
    
    enum INPUT_MODE {
        INIT,
        CIRC_PTS,
        IGNR_PTS,
        EXIT
    };
    
    struct INPUT_DATA {
        cv::Point2d cursorPt;
        std::vector<cv::Point2d> circPts;
        std::vector<std::vector<cv::Point2d> > ignrPts;
        INPUT_MODE mode;
        
        INPUT_DATA() {
            mode =  INIT;
            cursorPt.x = -1;
            cursorPt.y = -1;
            addPoly();
        }
        
        void addPoly() {
            ignrPts.push_back(std::vector<cv::Point2d>());
        }
    };
    
private:
    bool _open;
    std::string _config_fn;
    ConfigParser _cfg;
    cv::Mat _frame;
    size_t _w, _h;
    CameraModelPtr _cam_model;
    INPUT_DATA _input_data;
};
