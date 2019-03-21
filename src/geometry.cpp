/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Geometry.cpp
/// \brief      Geometry and vector algebra methods.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "geometry.h"

#include "CameraModel.h"
#include "SquareRT.h"
#include "Logger.h"

#include <opencv2/opencv.hpp>

#include <cmath>    // sqrt, cos, sin
#include <vector>

using cv::Mat;
using cv::Point;
using cv::Point2d;
using cv::Rect;
using std::vector;
using std::string;

///
/// Fit a plane to 3D points using the SVD method.
///
bool planeFit_SVD(const vector<CmPoint>& points, CmPoint& normal)
{
    CmPoint c;
    int n = static_cast<int>(points.size());
    
    if (n < 3) {
        LOG_ERR("Error! At least 3 points are needed to fit a plane (n = %d)!", n);
        return false;
    }
    
    // compute CoM
    for (auto p : points) { c += p; }
    if (n > 0) { c /= n; }
    
    // subtract CoM
    cv::Mat pts(n,3,CV_64F);
    for (int i = 0; i < n; i++) {
        CmPoint tmp = points[i] - c;
        pts.at<double>(i,0) = tmp.x;
        pts.at<double>(i,1) = tmp.y;
        pts.at<double>(i,2) = tmp.z;
    }
    
    // svd magic
    cv::Mat w, u, vt;
    cv::SVD::compute(pts, w, u, vt, cv::SVD::MODIFY_A);
    normal.x = vt.at<double>(2,0);
    normal.y = vt.at<double>(2,1);
    normal.z = vt.at<double>(2,2);
    normal.normalise();
    
    return true;
}

///
/// Project 2D image coords to 3D coords on the unit sphere.
///
size_t project2dPointsToUnitSphere(const vector<Point2d>& pts2d, const CameraModelPtr cam_model, vector<CmPoint>& pts3d)
{
    double vec[3];
    pts3d.clear();
    for (auto p2d : pts2d) {
        if (!cam_model->pixelToVector(p2d.x, p2d.y, vec)) {
            continue;
        }
        CmPoint tmp(vec[0], vec[1], vec[2]);
        tmp.normalise();
        pts3d.push_back(tmp);
    }
    return pts3d.size();
}

///
/// Fit circle to pixels, taking camera model into account.
///
bool circleFit_camModel(const vector<Point2d>& pix2d, const CameraModelPtr cam_model, CmPoint& c, double& r)
{
    /// Project points to unit sphere
    vector<CmPoint> pts3d;
    size_t n = project2dPointsToUnitSphere(pix2d, cam_model, pts3d);
    
    /// Fit plane
    if (!planeFit_SVD(pts3d, c)) {
        LOG_ERR("Error! Could not fit plane to 3D points!");
        return false;
    }
    
    /// Find average radial angle to circumference points
    /// The plane normal defines the central axis.
    r = 0;
    for (auto pt : pts3d) { r += pt.getAngleTo(c); }
    if (n > 0) { r /= n; }
    
    return true;
}

///
/// Compute animal-camera R+t transform from supplied square corners.
/// R is animal frame to camera frame transform.
///
bool computeRtFromSquare(const CameraModelPtr cam_model, const Mat& ref_cnrs, const vector<Point2d>& cnrs, Mat& R, Mat& t)
{
    if ((ref_cnrs.rows != 3) || (ref_cnrs.cols != 4) || (cnrs.size() != 4)) {
        LOG_ERR("Error! Invalid format for computing Rt matrices from square corners (ref=%dx%d, n=%d)!", ref_cnrs.cols, ref_cnrs.rows, cnrs.size());
        return false;
    }
    
    /// Project square corners.
    double vec[3];
    vector<CmPoint> cnr_vecs;
    for (int i = 0; i < 4; i++) {
        if (!cam_model->pixelToVector(cnrs[i].x, cnrs[i].y, vec)) {
            LOG_ERR("Error finding square homography! Input points were invalid (%f, %f).", cnrs[i].x, cnrs[i].y);
            return false;
        }
        vec3normalise(vec);
        cnr_vecs.push_back(CmPoint(vec[0], vec[1], vec[2]));
    }

    /// Minimise transform from reference corners.
    SquareRT square(cnr_vecs, ref_cnrs);
    double guess[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    if (!R.empty() && !t.empty()) {
        // init guess
        if ((R.depth() != CV_64F) || (t.depth() != CV_64F)) {
            LOG_ERR("Error finding square homography! R or t matrix has invalid depth (R=%f, t=%f).", R.depth(), t.depth());
            return false;
        }
        CmPoint64f angleAxis = CmPoint64f::matrixToOmega(R);
        for (int i = 0; i < 3; i++) {
            guess[i] = angleAxis[i];
            guess[3+i] = t.at<double>(i,0);
        }
    }
    square.optimize(guess);
    square.getOptX(guess);
    
    // double err = square.getOptF();
    // printf("Minimised R: %.6f %.6f %.6f   T: %.3f %.3f %.3f   (%.6f)\n",
        // guess[0], guess[1], guess[2], guess[3], guess[4], guess[5], err);
    
    /// Convert to mat.
    R = CmPoint64f::omegaToMatrix(CmPoint64f(guess[0], guess[1], guess[2]));
    t = (cv::Mat_<double>(3,1) << guess[3], guess[4], guess[5]);
    
    return true;
}

///
/// Wrapper for computing animal-camera R+t transform from XY square.
/// Square normal = animal Z axis. Corner ordering is TL (+X,-Y), TR (+X,+Y), BR (-X,+Y), BL (-X,-Y).
///
bool computeRtFromSquare_XY(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R, Mat& t)
{
    return computeRtFromSquare(cam_model, XY_CNRS, cnrs, R, t);
}

///
/// Wrapper for computing animal-camera R+t transform from YZ square.
/// Square normal = animal X axis. Corner ordering is TL (-Y,-Z), TR (+Y,-Z), BR (+Y,+Z), BL (-Y,+Z).
///
bool computeRtFromSquare_YZ(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R, Mat& t)
{
    return computeRtFromSquare(cam_model, YZ_CNRS, cnrs, R, t);
}

///
/// Wrapper for computing animal-camera R+t transform from XZ square.
/// Square normal = animal Y axis. Corner ordering is TL (+X,-Z), TR (-X,-Z), BR (-X,+Z), BL (+X,+Z).
///
bool computeRtFromSquare_XZ(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R, Mat& t)
{
    return computeRtFromSquare(cam_model, XZ_CNRS, cnrs, R, t);
}

///
/// Wrapper for computing animal-camera R+t transform.
///
bool computeRtFromSquare(const CameraModelPtr cam_model, const string ref_str, const vector<Point2d>& cnrs, Mat& R, Mat& t)
{
    bool ret = false;
    if (ref_str == "xy") {
        ret = computeRtFromSquare_XY(cam_model, cnrs, R, t);
    }
    else if (ref_str == "yz") {
        ret = computeRtFromSquare_YZ(cam_model, cnrs, R, t);
    }
    else if (ref_str == "xz") {
        ret = computeRtFromSquare_XZ(cam_model, cnrs, R, t);
    }
    return ret;
}