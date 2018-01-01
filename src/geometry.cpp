/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Geometry.cpp
/// \brief      Geometry and vector algebra methods.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "geometry.h"

#include "CameraModel.h"

#include <opencv2/opencv.hpp>

#include <boost/log/trivial.hpp>

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
    int n = points.size();
    
    if (n < 3) {
        BOOST_LOG_TRIVIAL(error) << "Error! At least 3 points are needed to fit a plane (n=" << n << ")!";
        return false;
    }
    
    // compute CoM
    for (auto p : points) { c += p; }
    if (n > 0) { c /= n; }
    
    // subtract CoM
    Mat pts(n,3,CV_32FC1);
    for (int i = 0; i < n; i++) {
        CmPoint tmp = points[i] - c;
        pts.at<float>(i,0) = tmp.x;
        pts.at<float>(i,1) = tmp.y;
        pts.at<float>(i,2) = tmp.z;
    }
    
    // svd magic
    Mat w, u, vt;
    cv::SVD::compute(pts, w, u, vt, cv::SVD::MODIFY_A);
    
    normal.x = vt.at<float>(2,0);
    normal.y = vt.at<float>(2,1);
    normal.z = vt.at<float>(2,2);
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
    int n = project2dPointsToUnitSphere(pix2d, cam_model, pts3d);
    
    /// Fit plane
    if (!planeFit_SVD(pts3d, c)) {
        BOOST_LOG_TRIVIAL(error) << "Error! Could not fit plane to 3D points!";
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
/// Compute camera-animal R transform from supplied square corners.
/// Alg from http://nghiaho.com/?page_id=671
///
bool computeRFromSquare(const CameraModelPtr cam_model, const Mat& ref_cnrs, const vector<Point2d>& cnrs, Mat& R)
{
    assert((ref_cnrs.rows == 3) && (ref_cnrs.cols == 4));
    assert(cnrs.size() == 4);
    
    /// Get view vectors for each corner and compute centroid.
    double vec[3];
    Mat tst_cnrs(4,3,CV_32FC1);
    Mat tst_cntr = Mat::zeros(1,3, CV_32FC1);
    Mat ref_cntr = Mat::zeros(3,1, CV_32FC1);
    for (int i = 0; i < 4; i++) {
        if (!cam_model->pixelToVector(cnrs[i].x, cnrs[i].y, vec)) {
            BOOST_LOG_TRIVIAL(error) << "Error finding square homography! Input points were invalid (" << cnrs[i].x << ", " << cnrs[i].y << ").";
            return false;
        }
        vec3normalise(vec);
        tst_cnrs.at<float>(i,0) = vec[0];
        tst_cnrs.at<float>(i,1) = vec[1];
        tst_cnrs.at<float>(i,2) = vec[2];
        
        tst_cntr += tst_cnrs(Rect(0,i,3,1));
        ref_cntr += ref_cnrs(Rect(i,0,1,3));
    }
    tst_cntr /= 4;
    ref_cntr /= 4;
    
    /// Compile covariance matrix
    Mat cov = Mat::zeros(3,3,CV_32FC1);
    for (int i = 0; i < 4; i++) {
        cov += ref_cnrs(Rect(i,0,1,3)) * (tst_cnrs(Rect(0,i,3,1)) - tst_cntr);
    }
    
    /// Compute SVD.
    Mat w, u, vt;
    cv::SVD::compute(cov, w, u, vt, cv::SVD::MODIFY_A);
    
    /// Compute R matrix (animal -> cam).
    R = vt*u;
    
    /// Check for flip.
    double det = cv::determinant(R);
    if (det < 0) {
        BOOST_LOG_TRIVIAL(debug) << "Flipping R";
        R(Rect(2,0,1,3)) = -R(Rect(2,0,1,3));
    }
    
    return true;
}

///
/// Wrapper for computing camera-animal R transform from XY square.
/// Square normal = animal Z axis. Corner ordering is TL (+X,-Y), TR (+X,+Y), BR (-X,+Y), BL (-X,-Y).
///
bool computeRFromSquare_XY(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R)
{
    /// Reference square corners - tl,tr,br,bl column-wise values.
    // TL (+X,-Y), TR (+X,+Y), BR (-X,+Y), BL (-X,-Y)
    static const Mat ref_cnrs = (cv::Mat_<float>(3,4) << 0.5, 0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0);
    
    return computeRFromSquare(cam_model, ref_cnrs, cnrs, R);
}

///
/// Wrapper for computing camera-animal R transform from YZ square.
/// Square normal = animal X axis. Corner ordering is TL (-Y,-Z), TR (+Y,-Z), BR (+Y,+Z), BL (-Y,+Z).
///
bool computeRFromSquare_YZ(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R)
{
    /// Reference square corners - tl,tr,br,bl column-wise values.
    // TL (-Y,-Z), TR (+Y,-Z), BR (+Y,+Z), BL (-Y,+Z)
    static const Mat ref_cnrs = (cv::Mat_<float>(3,4) << 0.0, 0.0, 0.0, 0.0, -0.5, 0.5, 0.5, -0.5, -0.5, -0.5, 0.5, 0.5);
    
    return computeRFromSquare(cam_model, ref_cnrs, cnrs, R);
}

///
/// Wrapper for computing camera-animal R transform from XZ square.
/// Square normal = animal Y axis. Corner ordering is TL (+X,-Z), TR (-X,-Z), BR (-X,+Z), BL (+X,+Z).
///
bool computeRFromSquare_XZ(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R)
{
    /// Reference square corners - tl,tr,br,bl column-wise values.
    // TL (+X,-Z), TR (-X,-Z), BR (-X,+Z), BL (+X,+Z)
    static const Mat ref_cnrs = (cv::Mat_<float>(3,4) << 0.5, -0.5, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, -0.5, -0.5, 0.5, 0.5);
    
    return computeRFromSquare(cam_model, ref_cnrs, cnrs, R);
}
