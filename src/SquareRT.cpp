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
Mat getCornerVecs(const double x[6], const Mat& ref_cnrs)
{
    /// Parse R+T.
    Mat T = (cv::Mat_<double>(3,4) << x[0], x[0], x[0], x[0], x[1], x[1], x[1], x[1], x[2], x[2], x[2], x[2]);
    Mat R = angleAxisToMat(CmPoint(x[3], x[4], x[5]));
    
    /// Apply R+T.
    return (R * ref_cnrs + T);
}

///
///
///
SquareRT::SquareRT(const vector<CmPoint>& cnrs, const Mat& ref_cnrs)
:	_corners(cnrs), _ref_corners(ref_cnrs)
{
    // tx ty tz rx ry rz
    double lb[6] = {-1e3, -1e3, 0, -CM_PI, -CM_PI, -CM_PI};
    double ub[6] = {1e3, 1e3, 1e3, CM_PI, CM_PI, CM_PI};
    init(NLOPT_LN_BOBYQA, 6);
    setLowerBounds(lb);
    setUpperBounds(ub);
    setXtol(1e-6);
    setMaxEval(1e4);

    double init_step[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    setInitialStep(init_step);
}

///
///
///
double SquareRT::objective(unsigned n, const double* x, double* grad)
{
    Mat cnrs = getCornerVecs(x, _ref_corners);
    double err = 0;
    for (int i = 0; i < 4; i++) {
        err += (_corners[i] - mat2vec(cnrs(cv::Rect(i,0,1,3))).getNormalised()).len2();
    }
    
    // printf("%d:\tT: %.2f %.2f %.2f  R: %.2f %.2f %.2f   (%f)\n", _nEval, x[0], x[1], x[2], x[3], x[4], x[5], err);
    
    return err;
}
