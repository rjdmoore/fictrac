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
    /// Data.
    struct DATA {
        // trackball state
        unsigned int cnt, seq;
        CmPoint64f dr_roi, r_roi;
        cv::Mat R_roi;
        CmPoint64f dr_cam, r_cam;
        cv::Mat R_cam;
        CmPoint64f dr_lab, r_lab;
        cv::Mat R_lab;
        double ts;

        double velx, vely, step_mag, step_dir, intx, inty, heading, posx, posy;

        // testing
        double dist, ang_dist, step_avg, step_var, evals_avg;

        // constructors
        DATA()
            : cnt(0), seq(0),
            dr_roi(CmPoint64f(0, 0, 0)), r_roi(CmPoint64f(0, 0, 0)),
            dr_cam(CmPoint64f(0, 0, 0)), r_cam(CmPoint64f(0, 0, 0)),
            dr_lab(CmPoint64f(0, 0, 0)), r_lab(CmPoint64f(0, 0, 0)),
            ts(-1),
            velx(0), vely(0),
            step_mag(0), step_dir(0),
            intx(0), inty(0),
            heading(0), posx(0), posy(0),
            dist(0), ang_dist(0),
            step_avg(0), step_var(0),
            evals_avg(0)
        {
            R_roi = cv::Mat::eye(3, 3, CV_64F);
            R_cam = cv::Mat::eye(3, 3, CV_64F);
            R_lab = cv::Mat::eye(3, 3, CV_64F);
        }

        DATA(const DATA &d)
            : cnt(d.cnt), seq(d.seq),
            dr_roi(d.dr_roi), r_roi(d.r_roi),
            dr_cam(d.dr_cam), r_cam(d.r_cam),
            dr_lab(d.dr_lab), r_lab(d.r_lab),
            ts(d.ts),
            velx(d.velx), vely(d.vely),
            step_mag(d.step_mag), step_dir(d.step_dir),
            intx(d.intx), inty(d.inty),
            heading(d.heading), posx(d.posx), posy(d.posy),
            dist(d.dist), ang_dist(d.ang_dist),
            step_avg(d.step_avg), step_var(d.step_var),
            evals_avg(d.evals_avg)
        {
            R_roi = d.R_roi.clone();
            R_cam = d.R_cam.clone();
            R_lab = d.R_lab.clone();
        }
    };

public:
    Trackball(std::string cfg_fn);
    ~Trackball();

    bool isActive() { return _active; }
    void terminate() { _kill = true; }
    std::shared_ptr<Trackball::DATA> getState();
    void dumpState();
    bool writeTemplate(std::string fn = "");

private:
    /// Worker function.
    void process();

    void resetData();
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
    std::shared_ptr<std::vector<double>> _p1s_lut;

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
    bool _do_global_search;
    int _max_bad_frames;
    int _nevals;

    /// Program.
    bool _init, _reset, _clean_map;

    /// Data
    DATA _data;

    /// Data i/o.
    std::string _base_fn;
    std::unique_ptr<FrameGrabber> _frameGrabber;
    bool _do_sock_output, _do_com_output;
    std::unique_ptr<Recorder> _data_log, _data_sock, _data_com;

    /// Thread stuff.
    std::atomic_bool _active, _kill, _do_reset;
    std::unique_ptr<std::thread> _thread;
};
