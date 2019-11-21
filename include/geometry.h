/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Geometry.h
/// \brief      Geometry and vector algebra methods.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "typesvars.h"
#include "SharedPointers.h"

#include <opencv2/opencv.hpp>

#include <cmath>

SHARED_PTR(CameraModel);


static inline CmReal vec3dot(const CmReal a[3], const CmReal b[3])
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline void vec3cross(const CmReal a[3], const CmReal b[3], CmReal v[3])
{
	v[0] = a[1]*b[2] - a[2]*b[1];
	v[1] = a[2]*b[0] - a[0]*b[2];
	v[2] = a[0]*b[1] - a[1]*b[0];
}

// returns magnitude of resulting vector
static CmReal vec3crossMag(const CmReal a[3], const CmReal b[3])
{
	CmReal v[3];
	vec3cross(a, b, v);
	return sqrt(vec3dot(v,v));
}

static inline void vec3scale(CmReal a[3], CmReal s)
{
	a[0] *= s;
	a[1] *= s;
	a[2] *= s;
}

static inline void vec3scale(const CmReal a[3], CmReal b[3], CmReal s)
{
	b[0] = a[0] * s;
	b[1] = a[1] * s;
	b[2] = a[2] * s;
}

static CmReal vec3normal(CmReal a[3])
{
	return sqrt(vec3dot(a,a));
}

static CmReal vec3normalise(CmReal a[3])
{
	CmReal mag = vec3normal(a);
	if (mag != 0)
		vec3scale(a, 1.0/mag);
	return mag;
}

///
/// A simple clamp function.
///
template<typename T>
static inline T clamp(T x, T min, T max)
{
    return (x<=min) ? min : (x>=max) ? max : x;
}

static inline cv::Mat vec2mat(const CmPoint vec)
{
    return (cv::Mat_<double>(3,1) << vec[0], vec[1], vec[2]);
}

static inline CmPoint mat2vec(const cv::Mat& mat)
{
    assert((mat.depth() == CV_64F) && (mat.rows == 3) && (mat.cols == 1));
    return CmPoint(mat.at<double>(0,0), mat.at<double>(1,0), mat.at<double>(2,0));
}

///
/// configGui helper functions
///

/// Fit a plane to 3D points using the SVD method.
bool planeFit_SVD(const std::vector<CmPoint>& points, CmPoint& normal);

/// Project 2D image coords to 3D coords on the unit sphere.
size_t project2dPointsToUnitSphere(const std::vector<cv::Point2d>& pts2d, const CameraModelPtr cam_model, std::vector<CmPoint>& pts3d);

/// Fit circle to pixels, taking camera model into account.
bool circleFit_camModel(const std::vector<cv::Point2d>& pix2d, const CameraModelPtr cam_model, CmPoint& c, double& r);

/// Reference square corners - tl,tr,br,bl column-wise values.
// TL (+X,-Y), TR (+X,+Y), BR (-X,+Y), BL (-X,-Y)
static const cv::Mat XY_CNRS = (cv::Mat_<double>(3,4) << 0.5, 0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0);
// TL (-Y,-Z), TR (+Y,-Z), BR (+Y,+Z), BL (-Y,+Z)
static const cv::Mat YZ_CNRS = (cv::Mat_<double>(3,4) << 0.0, 0.0, 0.0, 0.0, -0.5, 0.5, 0.5, -0.5, -0.5, -0.5, 0.5, 0.5);
// TL (+X,-Z), TR (-X,-Z), BR (-X,+Z), BL (+X,+Z)
static const cv::Mat XZ_CNRS = (cv::Mat_<double>(3,4) << 0.5, -0.5, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, -0.5, -0.5, 0.5, 0.5);

/// Compute camera-animal R+t transform from supplied square corners.
bool computeRtFromSquare(const CameraModelPtr cam_model, const cv::Mat& ref_cnrs, const std::vector<cv::Point2d>& cnrs, cv::Mat& R, cv::Mat& t);

/// Compute camera-animal R+t transform from XY square.
bool computeRtFromSquare_XY(const CameraModelPtr cam_model, const std::vector<cv::Point2d>& cnrs, cv::Mat& R, cv::Mat& t);

/// Compute camera-animal R+t transform from YZ square.
bool computeRtFromSquare_YZ(const CameraModelPtr cam_model, const std::vector<cv::Point2d>& cnrs, cv::Mat& R, cv::Mat& t);

/// Compute camera-animal R+t transform from XZ square.
bool computeRtFromSquare_XZ(const CameraModelPtr cam_model, const std::vector<cv::Point2d>& cnrs, cv::Mat& R, cv::Mat& t);

/// Compute camera-animal R+t transform.
bool computeRtFromSquare(const CameraModelPtr cam_model, const std::string ref_str, const std::vector<cv::Point2d>& cnrs, cv::Mat& R, cv::Mat& t);
