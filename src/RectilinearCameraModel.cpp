/// FicTrac http://rjdmoore.net/fictrac/
/// \file       RectilinearCameraModel.h
/// \brief      Implementation of a rectilinear camera model.
/// \author     Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#include "typesvars.h"
#include "CameraModel.h"

class RectilinearCameraModel : public CameraModel
{
public:
	RectilinearCameraModel(
		int width, int height, CmReal verticalFOV,
		CmReal centreX=-1, CmReal centreY=-1, CmReal imageCircleFOV=0);
	virtual bool pixelToVector(CmReal x, CmReal y, CmReal direction[3]) const;
	virtual bool vectorToPixel(const CmReal point[3], CmReal& x, CmReal& y) const;
	virtual bool validPixel(CmReal x, CmReal y) const;
	virtual CmReal getFOV() const { return _verticalFOV; }

private:
	CmReal _xc, _yc;
	CmReal _verticalFOV;
	CmReal _imageCircleFOV;
	CmReal _focalLengthPixels;
	CmReal _imageCircleR2;
	bool _validPixel(CmReal x, CmReal y, CmReal R2) const {
		return (R2 <= _imageCircleR2) && _validXY(x,y);
	}
};

CameraModelPtr CameraModel::createRectilinear(
		int width, int height, CmReal verticalFOV,
		CmReal centreX, CmReal centreY, CmReal imageCircleFOV)
{
	return CameraModelPtr(
		new RectilinearCameraModel(
			width, height, verticalFOV,
			centreX, centreY, imageCircleFOV));
}

RectilinearCameraModel::RectilinearCameraModel(
	int width, int height, CmReal verticalFOV,
	CmReal centreX, CmReal centreY, CmReal imageCircleFOV)
	: CameraModel(width, height)
{
	_imageCircleFOV = imageCircleFOV;
	_verticalFOV = verticalFOV;
	_xc = (centreX==-1) ? width*0.5 : centreX;
	_yc = (centreY==-1) ? height*0.5 : centreY;
	_focalLengthPixels = (height * 0.5) / tan(verticalFOV * 0.5);
	CmReal R = _focalLengthPixels * tan(imageCircleFOV * 0.5);
	if (imageCircleFOV <= 0)
		R = width + height; // allows everything
	_imageCircleR2 = R * R;
}

bool RectilinearCameraModel::pixelToVector(
	CmReal x, CmReal y, CmReal direction[3]) const
{
	// funny how simple it is :^)
	CmReal dx = x - _xc;
	CmReal dy = y - _yc;
	direction[0] = dx;
	direction[1] = dy;
	direction[2] = _focalLengthPixels;
	return _validPixel(x, y, dx*dx + dy*dy);
}

bool RectilinearCameraModel::vectorToPixel(
	const CmReal point[3], CmReal& x, CmReal& y) const
{
	if (point[2] <= 0.0) {
		// Uh oh, undefined - causes negative image behind viewer
		x = -1.0;
		y = -1.0;
		return false;
	}
	CmReal s = _focalLengthPixels / point[2];
	CmReal dx = point[0] * s;
	CmReal dy = point[1] * s;
	x = dx + _xc;
	y = dy + _yc;
	return _validPixel(x, y, dx*dx + dy*dy);
}

bool RectilinearCameraModel::validPixel(CmReal x, CmReal y) const
{
	CmReal dx = x - _xc;
	CmReal dy = y - _yc;
	return _validPixel(x, y, dx*dx + dy*dy);
}
