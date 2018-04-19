/// FicTrac http://rjdmoore.net/fictrac/
/// \file       EquiareaCameraModel.cpp
/// \brief      Implementation of an equi-area camera model.
/// \author     Saul Thurrowgood, Richard Moore
/// \copyright  CC BY-NC-SA 3.0


#include "typesvars.h"
#include "CameraModel.h"
#include "geometry.h"


///
/// Equi-area camera model.
/// Uses Gall-Peters projection for equal area (sort of, probably, etc).
///
class EquiAreaCameraModel : public CameraModel
{
public:
	EquiAreaCameraModel(
		int width, int height,
		CmReal latTop, CmReal latExtent,
		CmReal lonLeft, CmReal lonExtent);
	virtual bool pixelToVector(CmReal x, CmReal y, CmReal direction[3]) const;
	virtual bool vectorToPixel(const CmReal point[3], CmReal& x, CmReal& y) const;
	/// validPixel() is any within the image area, so use default method

	///
	/// This is a strange one. Just returns latitude extent for now.
	///
	virtual CmReal getFOV() const {
		return _latExtent;
	}

private:
	CmReal _latTop, _latExtent;
	CmReal _lonLeft, _lonExtent;
	CmReal _latPerPixel, _latPixelsPerWrap;
	CmReal _lonPerPixel, _lonPixelsPerWrap;
};

///
/// Shared pointer constructor.
///
CameraModelPtr CameraModel::createEquiArea(
	int width, int height,
	CmReal latTop, CmReal latExtent,
	CmReal lonLeft, CmReal lonExtent)
{
	return CameraModelPtr(
		new EquiAreaCameraModel(
			width, height,
			latTop, latExtent,
			lonLeft, lonExtent));
}

///
/// Constructor.
///
EquiAreaCameraModel::EquiAreaCameraModel(
	int width, int height,
	CmReal latTop, CmReal latExtent,
	CmReal lonLeft, CmReal lonExtent)
	: CameraModel(width, height),
	  _latTop(latTop), _latExtent(latExtent),
	  _lonLeft(lonLeft), _lonExtent(lonExtent)
{
	_latPerPixel = latExtent / height;
	_latPixelsPerWrap = fabs(CM_PI / _latPerPixel);
	_lonPerPixel = lonExtent / width;
	_lonPixelsPerWrap = fabs(2.0*CM_PI / _lonPerPixel);
}

///
/// Image to world transform.
/// Define (lat=0,lon=0) as forward.
///	-> Circles of latitude rotate around the Y-axis.
///
bool EquiAreaCameraModel::pixelToVector(
	CmReal x, CmReal y, CmReal direction[3]) const
{
	CmReal lat = y * _latPerPixel + _latTop;
	CmReal lon = x * _lonPerPixel + _lonLeft;
	direction[1] = -lat/CM_PI_2; /// vertical latitude component on neg Y-axis
	CmReal lonMag = sqrt(1.0-direction[1]*direction[1]);
	direction[0] = lonMag * sin(lon);
	direction[2] = lonMag * cos(lon);
	return _validXY(x,y);
}

///
/// World to image transform.
///
bool EquiAreaCameraModel::vectorToPixel(
	const CmReal point[3], CmReal& x, CmReal& y) const
{
	CmReal r[3] = {point[0], point[1], point[2]};
	vec3normalise(r);
	CmReal lat = -r[1]*CM_PI_2;
	CmReal lon = atan2(r[0], r[2]);
	CmReal plat = (lat - _latTop) / _latPerPixel;
	CmReal plon = (lon - _lonLeft) / _lonPerPixel;

	plon = fmod(plon, _lonPixelsPerWrap);
	x = (plon >= 0) ? plon : plon + _lonPixelsPerWrap;
	plat = fmod(plat, _latPixelsPerWrap);
	y = (plat >= 0) ? plat : plat + _latPixelsPerWrap;

	return _validXY(x,y);
}
