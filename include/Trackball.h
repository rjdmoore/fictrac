/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Trackball.h
/// \brief      Stores surface map and current orientation of tracking ball.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "typesvars.h"
#include "NLoptFunc.h"
#include "CameraModel.h"
#include "Recorder.h"
#include "FrameGrabber.h"
#include "ConfigParser.h"

#include <opencv2/opencv.hpp>

#include <memory>	// unique_ptr, shared_ptr
#include <thread>
#include <atomic>
#include <deque>
#include <vector>

///
/// Estimate track ball orientation and update surface map.
///
class Trackball : public NLoptFunc
{
public:
    Trackball(std::string cfg_fn);
    ~Trackball();

    bool trackingCompleted() { return _tracking_completed; }
    void printState();

private:
    /// Worker function.
    void process();

    void reset();
    double testRotation(const double x[3]);
    virtual double objective(unsigned n, const double* x, double* grad) { return testRotation(x); }
    bool doSearch(bool allow_global);
    void updateSphere();
    void updatePath();
    bool logData();
    void drawCanvas();

private:
    ConfigParser _cfg;

    /// Camera models and remapping.
    CameraModelPtr _src_model, _roi_model, _sphere_model;
    RemapTransformPtr _cam_to_roi;
    cv::Mat _roi_to_cam_R, _cam_to_lab_R;
    std::shared_ptr<double[]> _p1s_lut;

    /// Arrays.
    int _map_w, _map_h;
    int _roi_w, _roi_h;
    cv::Mat _src_frame, _roi_frame, _roi_mask;
    cv::Mat _sphere_map;

    /// Sphere vars.
    double _sphere_rad, _r_d_ratio;
    CmPoint64f _sphere_c;
    
    /// Optimisation.
    double _error_thresh, _bound, _err;
    bool _global_search;
    int _max_bad_frames;

    /// Program.
    bool _reset, _tracking_completed;

    /// Data.
    unsigned int _cnt, _seq;
    CmPoint64f _dr_roi, _r_roi;
    cv::Mat _R_roi;
    CmPoint64f _dr_cam, _r_cam;
    cv::Mat _R_cam;
    CmPoint64f _dr_lab, _r_lab;
    cv::Mat _R_lab;
    double _ts;

    double _velx, _vely, _step_mag, _step_dir, _intx, _inty, _heading, _posx, _posy;
    
    // test data
    double _dist, _ang_dist, _step_avg, _step_var;

    /// Data i/o.
    std::unique_ptr<FrameGrabber> _frameGrabber;
    std::unique_ptr<Recorder> _log;

    /// Thread stuff.
    std::atomic_bool _active;
    std::unique_ptr<std::thread> _thread;

    /// Display.
    bool _do_display;
    cv::Mat _view, _canvas;
    std::deque<cv::Mat> _R_roi_hist;
    std::vector<CmPoint64f> _pos_heading_hist;
};
