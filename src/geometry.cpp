/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Geometry.cpp
/// \brief      Geometry and vector algebra methods.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "geometry.h"

#include "CameraModel.h"
#include "SquareRT.h"

#include <opencv2/opencv.hpp>

#include <boost/log/trivial.hpp>

#include <cmath>    // sqrt, cos, sin
#include <vector>

using cv::Mat;
using cv::Point;
using cv::Point2d;
using cv::Rect;
using std::vector;
using std::string;

Mat angleUnitAxisToMat(const double angle, const CmPoint axis)
{
	double x=axis[0], y=axis[1], z=axis[2];
	double c = cos(angle);
	double d = 1.0 - c;
	double dx = d*x;
	double dy = d*y;
	double dz = d*z;
	double dxy = dx*y;
	double dxz = dx*z;
	double dyz = dy*z;
	double s = sin(angle);
	double sx = s*x;
	double sy = s*y;
	double sz = s*z;
    return (cv::Mat_<double>(3,3) << c + dx*x, dxy - sz, dxz + sy, dxy + sz, c + dy*y, dyz - sx, dxz - sy, dyz + sx, c + dz*z);
}

/// aka rodrigues()
Mat angleAxisToMat(const CmPoint angleAxis)
{
	double angle = angleAxis.len();
	if (angle <= 1e-7)
        return Mat::eye(3,3,CV_64F);
    return angleUnitAxisToMat(angle, angleAxis.getNormalised());
}

/// aka rodrigues()
//FIXME: remove branching.
CmPoint matToAngleAxis(const Mat& m)
{
    CmPoint angleAxis(0,0,0);
    
    assert((m.cols == 3) && (m.rows == 3));
    
    if (m.depth() == CV_32F) {
        // make sure m is not ill-conditioned
        double angle = acos(clamp((m.at<float>(0,0)+m.at<float>(1,1)+m.at<float>(2,2)-1)/2.0, -1.0, 1.0));
        double sin_angle = sin(angle);
        if( sin_angle != 0 ) { angle /= 2.0*sin_angle; }
        angleAxis[0] = angle*(m.at<float>(2,1)-m.at<float>(1,2));
        angleAxis[1] = angle*(m.at<float>(0,2)-m.at<float>(2,0));
        angleAxis[2] = angle*(m.at<float>(1,0)-m.at<float>(0,1));
    } else if (m.depth() == CV_64F) {
        // make sure m is not ill-conditioned
        double angle = acos(clamp((m.at<double>(0,0)+m.at<double>(1,1)+m.at<double>(2,2)-1)/2.0, -1.0, 1.0));
        double sin_angle = sin(angle);
        if( sin_angle != 0 ) { angle /= 2.0*sin_angle; }
        angleAxis[0] = angle*(m.at<double>(2,1)-m.at<double>(1,2));
        angleAxis[1] = angle*(m.at<double>(0,2)-m.at<double>(2,0));
        angleAxis[2] = angle*(m.at<double>(1,0)-m.at<double>(0,1));
    }
    return angleAxis;
}

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
/// Compute camera-animal R+t transform from supplied square corners.
///
bool computeRtFromSquare(const CameraModelPtr cam_model, const Mat& ref_cnrs, const vector<Point2d>& cnrs, Mat& R, Mat& t)
{
    assert((ref_cnrs.rows == 3) && (ref_cnrs.cols == 4));
    assert(cnrs.size() == 4);
    
    /// Project square corners.
    double vec[3];
    vector<CmPoint> cnr_vecs;
    for (int i = 0; i < 4; i++) {
        if (!cam_model->pixelToVector(cnrs[i].x, cnrs[i].y, vec)) {
            BOOST_LOG_TRIVIAL(error) << "Error finding square homography! Input points were invalid (" << cnrs[i].x << ", " << cnrs[i].y << ").";
            return false;
        }
        vec3normalise(vec);
        cnr_vecs.push_back(CmPoint(vec[0], vec[1], vec[2]));
    }

    /// Minimise transform from reference corners.
    SquareRT square(cnr_vecs, ref_cnrs);
    double guess[6] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    square.optimize(guess);
    square.getOptX(guess);
    double err = square.getOptF();

    // printf("Minimised T: %.3f %.3f %.3f   R: %.3f %.3f %.3f   (%.4f)\n",
        // guess[0], guess[1], guess[2], guess[3], guess[4], guess[5], err);
    
    /// Convert to mat.
    t = (cv::Mat_<double>(3,1) << guess[0], guess[1], guess[2]);
    R = angleAxisToMat(CmPoint(guess[3], guess[4], guess[5]));
    
    return true;
}

///
/// Wrapper for computing camera-animal R+t transform from XY square.
/// Square normal = animal Z axis. Corner ordering is TL (+X,-Y), TR (+X,+Y), BR (-X,+Y), BL (-X,-Y).
///
bool computeRtFromSquare_XY(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R, cv::Mat& t)
{
    return computeRtFromSquare(cam_model, XY_CNRS, cnrs, R, t);
}

///
/// Wrapper for computing camera-animal R+t transform from YZ square.
/// Square normal = animal X axis. Corner ordering is TL (-Y,-Z), TR (+Y,-Z), BR (+Y,+Z), BL (-Y,+Z).
///
bool computeRtFromSquare_YZ(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R, cv::Mat& t)
{
    return computeRtFromSquare(cam_model, YZ_CNRS, cnrs, R, t);
}

///
/// Wrapper for computing camera-animal R+t transform from XZ square.
/// Square normal = animal Y axis. Corner ordering is TL (+X,-Z), TR (-X,-Z), BR (-X,+Z), BL (+X,+Z).
///
bool computeRtFromSquare_XZ(const CameraModelPtr cam_model, const vector<Point2d>& cnrs, Mat& R, cv::Mat& t)
{
    return computeRtFromSquare(cam_model, XZ_CNRS, cnrs, R, t);
}
