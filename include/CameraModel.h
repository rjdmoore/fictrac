/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CameraModel.h
/// \brief      Parent class for converting between pixel coords and view vectors.
/// \author     Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#ifndef _CAMERA_MODEL_H
#define _CAMERA_MODEL_H 1

#include "typesvars.h"

#include "SharedPointers.h"
SHARED_PTR(CameraModel);
#include <boost/enable_shared_from_this.hpp>

#include <opencv2/opencv.hpp>

///
/// Basic camera model methods to convert between pixel coordinates
/// and real world directions in the camera reference frame. Pixel
/// coordinates are continuous i.e. (1,0) is the boundary between
/// the first pixel and the second pixel, and the top of the first
/// row of pixels.
///
/// Axes:  X-right  Y-down  Z-forward
///
class CameraModel
	: public boost::enable_shared_from_this<CameraModel>
{
public:
	typedef boost::shared_ptr<CameraModel> Ptr;

	///
	/// Must be virtual to allow proper subclass destruction.
	///
	virtual ~CameraModel() {}

	///
	/// Width and height of the pixel area.
	///
	int width() const { return _width; }
	int height() const { return _height; }

	///
	/// NOTE: getFOV() has a different meaning for each CameraModel.
	///
	virtual CmReal getFOV() const = 0;

	///
	/// Whether the pixel views any part of the world.
	/// Continuous coordinates e.g. (x == width) may be valid.
	///
	virtual bool validPixel(CmReal x, CmReal y) const {
		return _validXY(x,y);
	}
	bool validPixelIndex(int x, int y) const
	{
		CmReal fx=x, fy=y;
		CameraModel::indexToContinuous(fx, fy);
		return validPixel(fx, fy);
	}

	///
	/// Whether the world point is imaged by the camera.
	///
	virtual bool validVector(CmReal point[3]) const {
		CmReal x, y;
		return vectorToPixel(point, x, y);
	}
	bool validVector(CmPoint &point) const {
		CmReal x, y;
		return vectorToPixel((CmReal*)&point, x, y);
	}

	///
	/// Converts a sub-pixel image coordinate to a vector that points in
	/// the direction from which light is gathered. The returned direction
	/// vector may not be unit length. The reverse is performed by
	/// vectorToPixel(), which can be read as "world point to pixel".
	///
	virtual bool pixelToVector(
			CmReal x, CmReal y, CmReal direction[3]) const = 0;
	virtual bool vectorToPixel(
			const CmReal point[3], CmReal& x, CmReal& y) const = 0;

	bool pixelToVector(
			CmReal x, CmReal y, CmPoint &direction) const
	{
		return pixelToVector(x, y, (CmReal*)&direction);
	}
	bool vectorToPixel(
			const CmPoint &point, CmReal& x, CmReal& y) const
	{
		return vectorToPixel((CmReal*)&point, x, y);
	}

	///
	/// Same as above but converts to/from pixel array indices rather
	/// than working with continuous values i.e. integer array index
	/// [0,w-1] rather than continuous [0,w]. The pixel at the integer
	/// array index will be sampled at it's centre.
	///
	bool pixelIndexToVector(CmReal x, CmReal y, CmReal direction[3]) const
	{
		CameraModel::indexToContinuous(x, y);
		return pixelToVector(x, y, direction);
	}
	bool pixelIndexToVector(CmReal x, CmReal y, CmPoint &direction) const
	{
		return pixelIndexToVector(x, y, (CmReal *)&direction);
	}
	bool vectorToPixelIndex(const CmReal point[3], int& x, int& y) const
	{
		CmReal fx, fy;
		bool ret = vectorToPixelIndex(point, fx, fy);
		x = (int)(fx + (CmReal)0.5); /// add 0.5 for rounding to int
		y = (int)(fy + (CmReal)0.5);
		return ret;
	}
	bool vectorToPixelIndex(const CmReal point[3], CmReal& x, CmReal& y) const
	{
		bool ret = vectorToPixel(point, x, y);
		/// Convert back to index but retain subpixel.
		CameraModel::continuousToIndex(x, y);
		return ret;
	}
	bool vectorToPixelIndex(const CmPoint &point, int& x, int& y) const
	{
		return vectorToPixelIndex((const CmReal*)&point, x, y);
	}
	bool vectorToPixelIndex(const CmPoint &point, CmReal& x, CmReal& y) const
	{
		return vectorToPixelIndex((const CmReal*)&point, x, y);
	}

	///
	/// Convert a pixel index to a continuous pixel position.
	/// An integer index is treated as indicating the pixel centre.
	///
	static void indexToContinuous(CmReal& x, CmReal& y) {
		x += (CmReal)0.5;  y += (CmReal)0.5;
	}
	static void indexToContinuous(
		int ix, int iy, CmReal& cx, CmReal& cy)
	{
		cx = ix + (CmReal)0.5;  cy = iy + (CmReal)0.5;
	}
	static void indexToContinuous(
		CmReal ix, CmReal iy, CmReal& cx, CmReal& cy)
	{
		cx = ix + (CmReal)0.5;  cy = iy + (CmReal)0.5;
	}

	///
	/// Convert a continuous pixel position to a pixel index.
	///
	static void continuousToIndex(CmReal& x, CmReal& y) {
		x -= (CmReal)0.5;  y -= (CmReal)0.5;
	}
	static void continuousToIndex(
		CmReal& cx, CmReal& cy, int& ix, int& iy)
	{
		ix = static_cast<int>(cx - (CmReal)0.5);  iy = static_cast<int>(cy - (CmReal)0.5);
	}
	static void continuousToIndex(
		CmReal& cx, CmReal& cy, CmReal& ix, CmReal& iy)
	{
		ix = cx - (CmReal)0.5;  iy = cy - (CmReal)0.5;
	}

	///
	/// Models a perfect rectilinear lens image.
	///
	/// getFOV() returns the vertical FOV.
	///
	static Ptr createRectilinear(
		int width, int height, CmReal verticalFOV,
		CmReal centreX=-1, CmReal centreY=-1,
		CmReal imageCircleFOV=0);

	///
	/// Models a perfect f-theta lens image e.g. simulated fisheye image.
	///
	/// getFOV() returns the imageCircleFOV.
	///
	static Ptr createFisheye(
		int width, int height,
		CmReal radPerPixel, CmReal imageCircleFOV,
		CmReal centreX=-1, CmReal centreY=-1);

	///
	/// Models an equirectangular image e.g. for 360x180-degree panoramas.
	///
	/// Does not fully support latitudes outside the range [-pi/2, pi/2],
	/// particularly when used as the source of a remap i.e. vectorToPixel()
	/// because the information is lost once converted to a vector.
	///
	/// getFOV() returns the latitude extent for now.
	///
//	static Ptr createEquirectangular(
//		int width, int height,
//		CmReal latTop=CM_PI_2, CmReal latExtent=-CM_PI,
//		CmReal lonLeft=CM_PI, CmReal lonExtent=2*CM_PI);

	static Ptr createEquiArea(
		int width, int height,
		CmReal latTop=CM_PI_2, CmReal latExtent=-CM_PI,
		CmReal lonLeft=CM_PI, CmReal lonExtent=2*CM_PI);

protected:
	int _width, _height;

	CameraModel(int width, int height);

	bool _validXY(CmReal x, CmReal y) const {
		return !((x < 0) || (x > _width) || (y < 0) || (y > _height));
	}
};

#endif // _CAMERA_MODEL_H
