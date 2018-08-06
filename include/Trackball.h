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

#include <opencv2/opencv.hpp>

#include <memory>	// unique_ptr

///
/// Estimate track ball orientation and update surface map.
///
class Trackball : public NLoptFunc
{
public:
    Trackball(  int             sphere_map_w,
                int             sphere_map_h,
                CameraModelPtr  roi_model,
                CmPoint64f&     roi_to_cam_r,
                CmPoint64f&     cam_to_lab_r,
                double          sphere_diam_rad,
                double          error_thresh,
                int             max_bad_frames,
                cv::Mat&        mask,
                cv::Mat&        sphere_template,
                bool            do_display,
                double          bound,
                double          tol,
                int             max_eval
    );
    ~Trackball();

    void reset();

    /// cv::Mat not copied - careful not to modify incoming mat until we're done with it here.
    bool update(cv::Mat view, double timestamp, bool allow_global);

    void drawDebug(cv::Mat& sphere, cv::Mat& orient);

    //const cv::Mat& get_R() { return _R_roi; }
    //const CmPoint64f& get_dR() { return _dr_roi; }
    //double get_err() { return _err; }
    //const cv::Mat& get_template() { return _sphere; }

private:
    double testRotation(const double x[3]);
    virtual double objective(unsigned n, const double* x, double* grad) { return testRotation(x); }
    void setCurrentView(cv::Mat view, double timestamp) { _roi = view; _ts = timestamp; }
    bool doSearch(bool allow_global);
    void updateSphere();
    void updatePath();
    bool logData();

private:
    /// Vars.
    int _map_w, _map_h;
    int _roi_w, _roi_h;
    double _r_d_ratio, _error_thresh, _bound, _err;
    int _max_bad_frames;
    bool _reset;

    CameraModelPtr _roi_model, _sphere_model;
    cv::Mat _sphere, _roi, _mask;
    cv::Mat _roi_to_cam_R, _cam_to_lab_R;
    cv::Mat _sphere_hist, _sphere_max;
    double _ts;

    std::shared_ptr<double[]> _p1s_lut;

    /// Data.
    unsigned int _cnt, _seq;
    CmPoint64f _dr_roi;
    cv::Mat _R_roi;
    CmPoint64f _dr_cam, _r_cam;
    cv::Mat _R_cam;
    CmPoint64f _dr_lab, _r_lab;
    cv::Mat _R_lab;

    double _velx, _vely, _step_mag, _step_dir, _intx, _inty, _heading, _posx, _posy;

    /// Data recording.
    std::unique_ptr<Recorder> _log;

    /// Display.
    cv::Mat _orient;
    bool _do_display;
};
