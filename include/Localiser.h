/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Localiser.h
/// \brief      Localises current sphere ROI within surface map.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "NLoptFunc.h"
#include "CameraModel.h"
#include "typesvars.h"

#include <opencv2/opencv.hpp>

#include <memory>   // shared_ptr
#include <vector>

///
///
///
class Localiser : public NLoptFunc
{
public:
    Localiser(nlopt_algorithm alg, double bound, double tol, int max_evals,
        CameraModelPtr sphere_model, const cv::Mat& sphere_map,
        const cv::Mat& roi_mask, std::shared_ptr<std::vector<double>> p1s_lut);
    ~Localiser() {};

    double search(cv::Mat& roi_frame, cv::Mat& R_roi, CmPoint64f& vx);

private:
    double testRotation(const double x[3]);
    virtual double objective(unsigned n, const double* x, double* grad) { return testRotation(x); }

private:
    double _bound;
    const double* _R_roi;
    CameraModelPtr _sphere_model;
    const cv::Mat _sphere_map, _roi_mask;
    std::shared_ptr<std::vector<double>> _p1s_lut;
    cv::Mat _roi_frame;
    int _roi_w, _roi_h;
};
