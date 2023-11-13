/// FicTrac http://rjdmoore.net/fictrac/
/// \file       ConfigGui.h
/// \brief      Interactive GUI for configuring FicTrac.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "typesvars.h"
#include "ConfigParser.h"
#include "FrameSource.h"

#include "SharedPointers.h"
SHARED_PTR(CameraModel);

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

class ConfigGui
{
public:
    ConfigGui(std::string config_fn, std::string src_override = "");
    ~ConfigGui();
    
    bool is_open();
    bool run();
    
    enum INPUT_MODE {
        CIRC_INIT,
        CIRC_PTS,
        IGNR_INIT,
        IGNR_PTS,
        R_INIT,
        R_SLCT,
        R_XY,
        R_YZ,
        R_XZ,
        R_MAN,
        R_EXT,
        EXIT
    };
	std::string INPUT_MODE_STR[12] = {
		"CIRC_INIT",
		"CIRC_PTS",
		"IGNR_INIT",
		"IGNR_PTS",
		"R_INIT",
		"R_SLCT",
		"R_XY",
		"R_YZ",
		"R_XZ",
		"R_MAN",
		"R_EXT",
		"EXIT"
	};
    
    struct INPUT_DATA {
        bool newEvent;
        cv::Point2d cursorPt;
        std::vector<cv::Point2d> circPts;
        std::vector<std::vector<cv::Point2d> > ignrPts;
        std::vector<cv::Point2d> sqrPts;
        INPUT_MODE mode;
        float ptScl;
        
        INPUT_DATA() {
            newEvent = false;
            mode =  CIRC_INIT;
            cursorPt.x = -1;
            cursorPt.y = -1;
            ptScl = -1;
            addPoly();
        }
        
        void addPoly() {
            ignrPts.push_back(std::vector<cv::Point2d>());
        }
    };

private:
    bool setFrame(cv::Mat& frame);

    bool updateRt(const std::string& ref_str, cv::Mat& R, cv::Mat& t);
    //void drawC2ATransform(cv::Mat& disp_frame, const cv::Mat& ref_cnrs, const cv::Mat& R, const cv::Mat& t, const double& r, const CmPoint& c);
    void drawC2AAxes(cv::Mat& disp_frame, const cv::Mat& R, const cv::Mat& t, const double& r, const CmPoint& c);
    void drawC2ACorners(cv::Mat& disp_frame, const std::string& ref_str, const cv::Mat& R, const cv::Mat& t);
    bool saveC2ATransform(const std::string& ref_str, const cv::Mat& R, const cv::Mat& t);

    void changeState(INPUT_MODE new_state);
    
private:
    std::string _config_fn, _base_fn;
    ConfigParser _cfg;
    int _w, _h;
    float _disp_scl;
    CameraModelPtr _cam_model;
    INPUT_DATA _input_data;

    std::shared_ptr<FrameSource> _source;
};
