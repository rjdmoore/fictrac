/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Trackball.h
/// \brief      Stores surface map and current orientation of tracking ball.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "typesvars.h"
#include "Localiser.h"
#include "CameraModel.h"
#include "Recorder.h"
#include "FrameGrabber.h"
#include "ConfigParser.h"

/// OpenCV individual includes required by gcc?
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <memory>	// unique_ptr, shared_ptr
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <deque>
#include <vector>

///
/// Estimate track ball orientation and update surface map.
///
class Trackball
{
public:
    Trackball(std::string cfg_fn);
    ~Trackball();

    bool isActive() { return _active; }
    void terminate() { _kill = true; }
    void printState();
    bool writeTemplate(std::string fn = "");

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

private:
    /// Drawing
    struct DrawData {
        cv::Mat src_frame, roi_frame, sphere_view, sphere_map;
        CmPoint64f dr_roi;
        cv::Mat R_roi;
        std::deque<cv::Mat> R_roi_hist;
        std::deque<CmPoint64f> pos_heading_hist;
    };

    bool updateCanvasAsync(std::shared_ptr<DrawData> data);
    void processDrawQ();
    void drawCanvas(std::shared_ptr<DrawData> data);

    std::vector<std::shared_ptr<DrawData>> _drawQ;
    std::mutex _drawMutex;
    std::condition_variable _drawCond;

    bool _do_display, _save_raw, _save_debug;
    cv::Mat _sphere_view;
    std::deque<cv::Mat> _R_roi_hist;
    std::deque<CmPoint64f> _pos_heading_hist;
    cv::VideoWriter _debug_vid, _raw_vid;

    std::unique_ptr<std::thread> _drawThread;

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
    cv::Mat _sphere_map, _sphere_template;

    /// Sphere vars.
    double _sphere_rad, _r_d_ratio;
    CmPoint64f _sphere_c;
    
    /// Optimisation.
    std::unique_ptr<Localiser> _localOpt, _globalOpt;
    double _error_thresh, _err;
    bool _global_search;
    int _max_bad_frames;
    int _nevals;

    /// Program.
    bool _init, _reset, _clean_map;

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
    double _dist, _ang_dist, _step_avg, _step_var, _evals_avg;

    /// Data i/o.
    std::string _base_fn;
    std::unique_ptr<FrameGrabber> _frameGrabber;
    std::unique_ptr<Recorder> _log;

    /// Thread stuff.
    std::atomic_bool _active, _kill;
    std::unique_ptr<std::thread> _thread;
};
