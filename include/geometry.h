/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Geometry.h
/// \brief      Geometry and vector algebra methods.
/// \author     Richard Moore, Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "typesvars.h"

#include "SharedPointers.h"
SHARED_PTR(CameraModel);

#include <opencv2/opencv.hpp>

#include <cmath>

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

static CmReal vec3normalise(CmReal a[3])
{
	CmReal mag = sqrt(vec3dot(a,a));
	if (mag != 0)
		vec3scale(a, 1.0/mag);
	return mag;
}

// static inline CmReal clamp(CmReal x, CmReal min, CmReal max)
// {
	// return (x<=min) ? min : (x>=max) ? max : x;
// }

///
/// A simple clamp function.
///
template<typename T>
static inline T clamp(T x, T min, T max)
{
    return (x<=min) ? min : (x>=max) ? max : x;
}

/// Fit a plane to 3D points using the SVD method.
bool planeFit_SVD(const std::vector<CmPoint>& points, CmPoint& normal);

/// Project 2D image coords to 3D coords on the unit sphere.
size_t project2dPointsToUnitSphere(const std::vector<cv::Point2d>& pts2d, const CameraModelPtr cam_model, std::vector<CmPoint>& pts3d);

/// Fit circle to pixels, taking camera model into account.
bool circleFit_camModel(const std::vector<cv::Point2d>& pix2d, const CameraModelPtr cam_model, CmPoint& c, double& r);
