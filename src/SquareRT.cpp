/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SquareRT.cpp
/// \brief      Minimise perspective R+T transformation of a square.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "SquareRT.h"

#include "typesvars.h"
#include "geometry.h"

#include <opencv2/opencv.hpp>

using cv::Mat;
using std::vector;

///
///
///
Mat getCornerVecs(const CmPoint& r, const CmPoint& t, const Mat& ref_cnrs)
{
    /// Parse R+T.
    Mat T = (cv::Mat_<double>(3,4) << t[0], t[0], t[0], t[0], t[1], t[1], t[1], t[1], t[2], t[2], t[2], t[2]);
    Mat R = CmPoint64f::omegaToMatrix(CmPoint(r[0], r[1], r[2]));
    
    /// Apply R+T.
    return (R * ref_cnrs + T);
}

///
///
///
SquareRT::SquareRT(const vector<CmPoint>& cnrs, const Mat& ref_cnrs)
:	_corners(cnrs), _ref_corners(ref_cnrs)
{
    // tx ty tz r_az r_el r_mag
    double lb[6] = {-CM_PI, -CM_PI, -CM_PI, -1e4, -1e4, 0};
    double ub[6] = {CM_PI, CM_PI, CM_PI, 1e4, 1e4, 1e4};
    init(NLOPT_GN_CRS2_LM, 6);
    setLowerBounds(lb);
    setUpperBounds(ub);
    setXtol(1e-9);
    setFtol(1e-9);
    setMaxEval(static_cast<unsigned int>(1e4));
    
}

///
///
///
double SquareRT::objective(unsigned n, const double* x, double* grad)
{
    // clamp rotation to +/- pi
    CmPoint64f r(x[0], x[1], x[2]);
    CmPoint64f t(x[3], x[4], x[5]);
    double angle = clamp(r.normalise(), -CM_PI, CM_PI);
    r *= angle;
    
    Mat cnrs = getCornerVecs(r, t, _ref_corners);
    double err = 0;
    for (int i = 0; i < 4; i++) {
        err += (_corners[i] - mat2vec(cnrs(cv::Rect(i,0,1,3))).getNormalised()).len2();
    }
    
    // printf("%d:\tR: %.2f %.2f %.2f  T: %.2f %.2f %.2f   (%f)\n", _nEval, x[0], x[1], x[2], x[3], x[4], x[5], err);
    
    return err;
}
