/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CameraRemap.h
/// \brief      Remap from one camera model to another.
/// \author     Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#pragma once


#include "CameraModel.h"
#include "typesvars.h"
#include "Remapper.h"
#include "SharedPointers.h"

#include <vector>

SHARED_PTR(CameraModel);
SHARED_PTR(RemapTransform);


///
/// Remap an image from one CameraModel to another, while honouring the pixel
/// validation of both the source and destination models.
///
/// Applications include:
///	> Undistorting a calibrated, real camera image into an ideal format.
///	> Converting a simulated image to match a real camera that you have.
///	> ...
///
class CameraRemap : public Remapper
{
public:

	CameraRemap(const CameraModelPtr& src, const CameraModelPtr& dst,
			const RemapTransformPtr& trans=RemapTransformPtr());

	CameraModelPtr& getSrc() { return _src; }
	CameraModelPtr& getDst() { return _dst; }

	///
	/// Recompute image mapping with a different transformation.
	/// Setting 'trans' to NULL uses no additional transformation.
	///
	void setTransform(RemapTransformPtr trans);

protected:
	CameraModelPtr _src, _dst;
	RemapTransformPtr _trans;
	std::vector<float> _mapX, _mapY;

	/// Implement Remapper interface.
	virtual float * _getMapX() { return &_mapX[0]; }
	virtual float * _getMapY() { return &_mapY[0]; }
};


///
/// Provides a transformation within CameraModel coordinates, to be performed
/// when mapping the 'dst' pixels to the corresponding 'src' pixels.  Notice
/// that this does in fact go in the reverse direction which ensures there
/// are no holes in the output, but also means the RemapTransform must also
/// be in the reverse direction, as indicated by the name inverseTransform()
/// which must go from 'dst' to 'src' location in the view sphere.
///
class RemapTransform
{
public:
	///
	/// Apply a transform on the 'dst' vector to create the 'src' vector.
	/// Note that 'dst' is an unnormalised vector, as returned by
	/// CameraModel::pixelToVector().
	///
	virtual CmPoint inverseTransform(CmPoint dst) = 0;
	virtual ~RemapTransform() {}
};


///
/// Example RemapTransform simply using a 3x3 matrix.
///
class MatrixRemapTransform : public RemapTransform
{
public:
	static RemapTransformPtr createFromOmega(CmPoint omega)
	{
		MatrixRemapTransform *t = new MatrixRemapTransform();
		omega.omegaToMatrix(t->_m);
		return RemapTransformPtr(t);
	}

	static RemapTransformPtr createFromMat(CmReal m[9])
	{
		MatrixRemapTransform *t = new MatrixRemapTransform(m);
		return RemapTransformPtr(t);
	}

	MatrixRemapTransform(const CmReal m[9])
	{
		for (int i=0; i<9; ++i)
			_m[i] = m[i];
	}

	CmPoint inverseTransform(CmPoint dst)
	{
		return dst.getTransformed(_m);
	}

private:
	MatrixRemapTransform() {}
	CmReal _m[9];
};
