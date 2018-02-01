/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Trackball.h
/// \brief      Stores surface map and current orientation of tracking ball.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "NLoptFunc.h"
#include "CameraModel.h"

#include <opencv2/opencv.hpp>

#include <boost\shared_array.hpp>

///
/// Estimate track ball orientation and update surface map.
///
class Trackball : public NLoptFunc
{
public:
    Trackball(  CameraModelPtr  cam_model,
                int             sphere_w,
                int             sphere_h,
                double          sphere_r_d_ratio,
                cv::Mat&        mask,
                cv::Mat&        sphere_template,
                bool            do_display,
                bool            do_update,
                double          lower_bound,
                double          upper_bound,
                double          tol,
                int             max_eval
    );
    ~Trackball();

    void setCurrentView(cv::Mat view) { _view = view; }

    void clearSphere();

    cv::Mat& updateSphere(CmPoint64f& angleAxis);

    double testRotation(const double x[3]);

    virtual double objective(unsigned n, const double* x, double* grad)
    {
        return testRotation(x);
    }

    void drawDebug(cv::Mat& sphere, cv::Mat& orient);

    cv::Mat& getR() { return _R; }
    cv::Mat& getTemplate() { return _sphere; }

private:
    CameraModelPtr _cam_model, _sphere_model;
    cv::Mat _sphere, _view, _mask, _R;
    cv::Mat _sphere_hist, _sphere_max;

    int _w, _h;
    int _view_w, _view_h;
    double _r;

    boost::shared_array<double> _p1s_lut;

    /// Display.
    cv::Mat _orient;
    bool _do_display;
};
