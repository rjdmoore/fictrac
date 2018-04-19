/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CameraRemap.cpp
/// \brief      Remap from one camera model to another.
/// \author     Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#include "CameraRemap.h"

#include "geometry.h"

#include <opencv2/highgui.hpp>

using namespace cv;


///
/// Constructor.
///
CameraRemap::CameraRemap(
	const CameraModelPtr& src,
	const CameraModelPtr& dst,
	const RemapTransformPtr& trans)
	: Remapper(src->width(), src->height(), dst->width(), dst->height()),
	_src(src), _dst(dst), _trans(trans)
{
	_mapX.resize(_dstW * _dstH);
	_mapY.resize(_dstW * _dstH);
	setTransform(_trans);
}

///
/// Recompute image mapping with a different transformation.
///
void CameraRemap::setTransform(RemapTransformPtr trans)
{
	for (int y = 0; y < _dstH; y++) {
		for (int x = 0; x < _dstW; x++) {
			CmPoint v;
			CmReal sx, sy;
			bool valid = false;
			do {
				if (!_dst->pixelIndexToVector(x, y, v))
					break;
				if (trans)
					v = trans->inverseTransform(v);
				if (!_src->vectorToPixelIndex(v, sx, sy))
					break;
				valid = true;
			} while (false);

			int i = y*_dstW + x;
			if (valid) {
				_mapX[i] = clamp<CmReal>(sx, 0.0, _srcW-1);
				_mapY[i] = clamp<CmReal>(sy, 0.0, _srcH-1);
			} else {
				/// Set a value outside of dst image ROI.
				_mapX[i] = INVALID_MAP_VAL;
				_mapY[i] = INVALID_MAP_VAL;
			}
		}
	}
}
