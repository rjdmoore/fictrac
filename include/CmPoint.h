/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CmPoint.h
/// \brief      3D vector type with several useful operations.
/// \author     Saul Thurrowgood, Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include <opencv2/opencv.hpp>

///
/// Typedef the types we support by explicit instantiation in CmPoint.cpp
///
template<typename T> class CmPointT;
typedef CmPointT<float>  CmPoint32f;
typedef CmPointT<double> CmPoint64f;

template<typename T>
class CmPointT {
public:
	T x, y, z;

	CmPointT() : x(0), y(0), z(0) {}
	CmPointT(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
	CmPointT(const CvPoint& p) : x(p.x), y(p.y), z(0) {}
	CmPointT(const CvPoint2D32f& p) : x(p.x), y(p.y), z(0) {}
	CmPointT(const CvPoint3D32f& p) : x(p.x), y(p.y), z(p.z) {}
	CmPointT(const CmPoint32f& p) : x(p.x), y(p.y), z(p.z) {}
	CmPointT(const CmPoint64f& p) : x(p.x), y(p.y), z(p.z) {}
    CmPointT(T az, T el);

	/// Allow implicit conversion of scalar to CmPointT for scaling
	CmPointT(T scale) : x(scale), y(scale), z(scale) {}

	void copyTo(CvPoint3D32f& p) const { p = cvPoint3D32f(x,y,z); }
	void copyTo(cv::Point3f& p) const { p = cv::Point3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)); }
	void copyTo(cv::Point3d& p) const { p = cv::Point3d(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)); }
	void copyTo(float *p) const { p[0] = static_cast<float>(x); p[1] = static_cast<float>(y); p[2] = static_cast<float>(z); }
	void copyTo(double *p) const { p[0] = static_cast<double>(x); p[1] = static_cast<double>(y); p[2] = static_cast<double>(z); }

	/// No implicit conversion from pointer types, so provide copy
	void copy(const float *p) { x = static_cast<T>(p[0]); y = static_cast<T>(p[1]); z = static_cast<T>(p[2]); }
	void copy(const double *p) { x= static_cast<T>(p[0]); y = static_cast<T>(p[1]); z = static_cast<T>(p[2]); }

	/// Allow indexing as if it were an array of T
	T& operator[] (unsigned i) { return (&x)[i]; }
	const T& operator[] (unsigned i) const { return (&x)[i]; }

	T normalise();
	CmPointT getNormalised() const;
	T len2() const { return x*x + y*y + z*z; }
	T len() const;

	///
	/// Returns the angle to point 'b' with the greatest precision by
	/// choosing between asin() or acos() depending on actual angle:
	///    asin() is bad around 90 degrees
	///    acos() is bad around 0 and 180 degrees
	///
	T getAngleTo(const CmPointT& b) const
	{
		return getNormalised().getAngleToNorm(b.getNormalised());
	}

	///
	/// Same as getAngleTo() but assumes both this vector and 'b' are unit.
	///
	T getAngleToNorm(const CmPointT& b) const;

	///
	/// Create a rotation matrix by interpreting this vector as a rotation
	/// axis whose magnitude gives the amount of rotation around the axis.
	/// Allow destination matrix to be single or double precision.
	///
	void omegaToMatrix(float R[9]) const;
	void omegaToMatrix(double R[9]) const;
    
    static cv::Mat_<T> omegaToMatrix(const CmPointT& omega);
    static CmPointT<T> matrixToOmega(const cv::Mat_<T>& m);
    
    void omegaToAzElMag(T& az, T& el, T& mag) const;

	///
	/// Multiply this point by a 3x3 matrix and return the result.
	///
    CmPointT getTransformed(cv::Mat_<T> m) const;

	CmPointT getTransformed(const T M[9]) const
	{
		return CmPointT(
			x*M[0] + y*M[1] + z*M[2],
			x*M[3] + y*M[4] + z*M[5],
			x*M[6] + y*M[7] + z*M[8]);
	}
	void transform(const T M[9]) { *this = getTransformed(M); }

	///
	/// Create a rotation around this vector by the amount 'angle'.
	///
	void getRotationAbout(T angle, T R[9]) const;
	void getRotationAboutNorm(T angle, T R[9]) const;

    ///
    /// Find rotation between this vetor and specified vector.
    ///
    CmPointT getRotationTo(CmPointT vec) const;
    CmPointT getRotationToNorm(CmPointT vec) const;

	///
	/// Rotates this point about an axis (or normalised axis).
	///
	void rotateAbout(const CmPointT& omega)
	{
		T R[9];
		omega.omegaToMatrix(R);
		transform(R);
	}
	CmPointT getRotatedAbout(const CmPointT& omega) const
	{
		CmPointT a = *this;
		a.rotateAbout(omega);
		return a;
	}

	void rotateAbout(const CmPointT& axis, T angle)
	{
		rotateAboutNorm(axis.getNormalised(), angle);
	}
	CmPointT getRotatedAbout(const CmPointT& axis, T angle) const
	{
		CmPointT a = *this;
		a.rotateAbout(axis, angle);
		return a;
	}

	void rotateAboutNorm(const CmPointT& axis, T angle)
	{
		T R[9];
		axis.getRotationAboutNorm(angle, R);
		transform(R);
	}
	CmPointT getRotatedAboutNorm(const CmPointT& axis, T angle) const
	{
		CmPointT a = *this;
		a.rotateAboutNorm(axis, angle);
		return a;
	}
    
    /// There are infinite possible orthogonal vectors, this finds one of them
    CmPointT getOrthVecNorm() const;
    
    void rotateAboutOrthVec(T angle);
    CmPointT getRotatedAboutOrthVec(T angle) const;

	///
	/// Operators for use in non-member wrappers - named to avoid
	/// ambiguities with those wrappers.
	///

	/// Component-wise multiply
	const CmPointT mul(const CmPointT& rhs) const
		{ return CmPointT(x*rhs.x, y*rhs.y, z*rhs.z); }
	/// Component-wise divide
	const CmPointT div(const CmPointT& rhs) const
		{ return CmPointT(x/rhs.x, y/rhs.y, z/rhs.z); }
	/// Vector add
	const CmPointT add(const CmPointT& rhs) const
		{ return CmPointT(x+rhs.x, y+rhs.y, z+rhs.z); }
	/// Vector subtract
	const CmPointT sub(const CmPointT& rhs) const
		{ return CmPointT(x-rhs.x, y-rhs.y, z-rhs.z); }
	/// Dot/scalar/inner product
	const T dot(const CmPointT& rhs) const
		{ return x*rhs.x + y*rhs.y + z*rhs.z; }
	/// Cross/vector/outer product
	const CmPointT crs(const CmPointT& rhs) const
		{ return CmPointT(
			y*rhs.z - z*rhs.y,
			z*rhs.x - x*rhs.z,
			x*rhs.y - y*rhs.x);
		}

	///
	/// Member operators that compute then assign-to this point.
	///
	CmPointT& operator*= (const CmPointT& rhs) { return (*this=mul(rhs)); }
	CmPointT& operator/= (const CmPointT& rhs) { return (*this=div(rhs)); }
	CmPointT& operator+= (const CmPointT& rhs) { return (*this=add(rhs)); }
	CmPointT& operator-= (const CmPointT& rhs) { return (*this=sub(rhs)); }
	CmPointT& operator^= (const CmPointT& rhs) { return (*this=crs(rhs)); }
};


///
/// Provide non-member functions to allow implicit conversion from other
/// data types to the CmPointT type.
///
/// Must create wrappers for non-member functions since C++ cannot implicitly
/// convert types that involve template parameters i.e. unspecified template
/// parameters are bad, but can implicitly convert when parameters are given.
///

inline const CmPoint32f operator* (const CmPoint32f& lhs, const CmPoint32f& rhs)
	{ return lhs.mul(rhs); }
inline const CmPoint32f operator/ (const CmPoint32f& lhs, const CmPoint32f& rhs)
	{ return lhs.div(rhs); }
inline const CmPoint32f operator+ (const CmPoint32f& lhs, const CmPoint32f& rhs)
	{ return lhs.add(rhs); }
inline const CmPoint32f operator- (const CmPoint32f& lhs, const CmPoint32f& rhs)
	{ return lhs.sub(rhs); }
inline const float operator% (const CmPoint32f& lhs, const CmPoint32f& rhs)
	{ return lhs.dot(rhs); }
inline const CmPoint32f operator^ (const CmPoint32f& lhs, const CmPoint32f& rhs)
	{ return lhs.crs(rhs); }
inline const CmPoint32f operator- (const CmPoint32f& p)
	{ return CmPoint32f(-p.x, -p.y, -p.z); }

inline const CmPoint64f operator* (const CmPoint64f& lhs, const CmPoint64f& rhs)
	{ return lhs.mul(rhs); }
inline const CmPoint64f operator/ (const CmPoint64f& lhs, const CmPoint64f& rhs)
	{ return lhs.div(rhs); }
inline const CmPoint64f operator+ (const CmPoint64f& lhs, const CmPoint64f& rhs)
	{ return lhs.add(rhs); }
inline const CmPoint64f operator- (const CmPoint64f& lhs, const CmPoint64f& rhs)
	{ return lhs.sub(rhs); }
inline const double operator% (const CmPoint64f& lhs, const CmPoint64f& rhs)
	{ return lhs.dot(rhs); }
inline const CmPoint64f operator^ (const CmPoint64f& lhs, const CmPoint64f& rhs)
	{ return lhs.crs(rhs); }
inline const CmPoint64f operator- (const CmPoint64f& p)
	{ return CmPoint64f(-p.x, -p.y, -p.z); }
