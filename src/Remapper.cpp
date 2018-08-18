/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Remapper.cpp
/// \brief      Base class for remapping images.
/// \author     Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#include "Remapper.h"

#include "typesvars.h"
#include "CameraModel.h"
#include "Logger.h"

/// OpenCV individual includes required by gcc?
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>


Remapper::Remapper(int srcW, int srcH, int dstW, int dstH)
	: _mode(LINEAR), _srcW(srcW), _srcH(srcH), _dstW(dstW), _dstH(dstH)
{
}

void Remapper::applyC1(
	const unsigned char *src, unsigned char *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_C1, src, dst, srcStep, dstStep);
}

void Remapper::applyC3(
	const unsigned char *src, unsigned char *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_C3, src, dst, srcStep, dstStep);
}

void Remapper::applyC4(
	const unsigned char *src, unsigned char *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_C4, src, dst, srcStep, dstStep);
}

void Remapper::applyC1(const char *src, char *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_C1, src, dst, srcStep, dstStep);
}

void Remapper::applyC3(const char *src, char *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_C3, src, dst, srcStep, dstStep);
}

void Remapper::applyC4(const char *src, char *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_C4, src, dst, srcStep, dstStep);
}

void Remapper::applyS1(
	const unsigned short *src, unsigned short *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_S1, src, dst, srcStep, dstStep);
}

void Remapper::applyS3(
	const unsigned short *src, unsigned short *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_S3, src, dst, srcStep, dstStep);
}

void Remapper::applyS4(
	const unsigned short *src, unsigned short *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_S4, src, dst, srcStep, dstStep);
}

void Remapper::applyF1(const float *src, float *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_F1, src, dst, srcStep, dstStep);
}

void Remapper::applyF3(const float *src, float *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_F3, src, dst, srcStep, dstStep);
}

void Remapper::applyF4(const float *src, float *dst, int srcStep, int dstStep)
{
	_apply(REMAP_TYPE_F4, src, dst, srcStep, dstStep);
}

void Remapper::apply(const IplImage *src, IplImage *dst)
{
	///
	/// Sanity check.
	///
	if (src->width!=_srcW || src->height!=_srcH) {
		LOG_ERR("Error applying remapping! Unexpected source image size (%dx%d)!", src->width, src->height);
		return;
	}
	if (dst->width!=_dstW || dst->height!=_dstH) {
		LOG_ERR("Error applying remapping! Unexpected destination image size (%dx%d)!", dst->width, dst->height);
		return;
	}

	///
	/// Note: not enforcing nChannels consistency directly, but a test is
	///       performed in _apply() but only to avoid segfaults.  This is
	///       to allow the likes of BayerRemap to overload the _apply()
	///       method and perform a remapping using a single channel source
	///       image and a three channel destination image.
	///
	if (src->depth != dst->depth) {
		LOG_ERR("Error applying remapping! Image depth mismatch (%d != %d).", src->depth, dst->depth);
		return;
	}

	switch (src->depth) {
	case IPL_DEPTH_8U:
	case IPL_DEPTH_8S:
		switch (src->nChannels) {
		case 1: _apply(REMAP_TYPE_C1,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		case 3: _apply(REMAP_TYPE_C3,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		case 4: _apply(REMAP_TYPE_C4,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		default: break;
		}
		break;
	case IPL_DEPTH_16U:
	case IPL_DEPTH_16S:
		switch (src->nChannels) {
		case 1: _apply(REMAP_TYPE_S1,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		case 3: _apply(REMAP_TYPE_S3,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		case 4: _apply(REMAP_TYPE_S4,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		default: break;
		}
		break;
	case IPL_DEPTH_32F:
		switch (src->nChannels) {
		case 1: _apply(REMAP_TYPE_F1,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		case 3: _apply(REMAP_TYPE_F3,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		case 4: _apply(REMAP_TYPE_F4,
					src->imageData, dst->imageData,
					src->widthStep, dst->widthStep); break;
		default: break;
		}
		break;
	default:
		LOG_ERR("Error applying remapping! Invalid data type");
		return;
	}
}

void Remapper::apply(const cv::Mat& src, cv::Mat& dst)
{
	///
	/// Ensure destination is same type as source.
	///
	if (!dst.data || dst.cols!=_dstW || dst.rows!=_dstH
			|| dst.type()!=src.type())
	{
		dst.create(_dstH, _dstW, src.type());
		dst.setTo(cv::Scalar::all(0));
	}

	IplImage iplSrc = src;
	IplImage iplDst = dst;
	apply(&iplSrc, &iplDst);
}


///
/// OpenCV 2.0+ version.
///

static inline cv::Mat _getCvMat(int type, const void *p, int w, int h, int step)
{
	if (step <= 0)
		step = cv::Mat::AUTO_STEP;
	return cv::Mat(h, w, type, const_cast<void*>(p), step);
}

void Remapper::_apply(
	RemapDataType type,
	const void *src, void *dst,
	int srcStep, int dstStep)
{
	///
	/// Sanity check to indirectly test number of channels are valid.
	///
	int bpp = _getBytesPerPixel(type);
	if ((srcStep>0 && (_srcW*bpp) > srcStep)
		|| (dstStep>0 && (_dstW*bpp) > dstStep)) {
		LOG_ERR("Error applying remapping! Failed channel sanity check!");
		return;
	}

	float *mapX = _getMapX();
	float *mapY = _getMapY();
	if (mapX==0 || mapY==0)
		return;
	cv::Mat mMapX(_dstH, _dstW, CV_32FC1, mapX);
	cv::Mat mMapY(_dstH, _dstW, CV_32FC1, mapY);

	int cvInterp;
	switch (_mode) {
	case NEAREST: cvInterp = cv::INTER_NEAREST; break;
	case LINEAR:  cvInterp = cv::INTER_LINEAR; break;
	case CUBIC:   cvInterp = cv::INTER_CUBIC; break;
// 	case LANCZOS: cvInterp = cv::INTER_LANCZOS4; break;
	default:
		LOG_ERR("Error applying remapping! Invalid interp mode");
		return;
	}

	int cvType;
	switch (type) {
	case REMAP_TYPE_C1: cvType=CV_8UC1; break;
	case REMAP_TYPE_C3: cvType=CV_8UC3; break;
	case REMAP_TYPE_C4: cvType=CV_8UC4; break;
	case REMAP_TYPE_S1: cvType=CV_16UC1; break;
	case REMAP_TYPE_S3: cvType=CV_16UC3; break;
	case REMAP_TYPE_S4: cvType=CV_16UC4; break;
	case REMAP_TYPE_F1: cvType=CV_32FC1; break;
	case REMAP_TYPE_F3: cvType=CV_32FC3; break;
	case REMAP_TYPE_F4: cvType=CV_32FC4; break;
	default:
		LOG_ERR("Error applying remapping! Invalid data type");
		return;
	}
	cv::Mat mSrc = _getCvMat(cvType, src, _srcW, _srcH, srcStep);
	cv::Mat mDst = _getCvMat(cvType, dst, _dstW, _dstH, dstStep);

	cv::remap(mSrc, mDst, mMapX, mMapY, cvInterp);
}
