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