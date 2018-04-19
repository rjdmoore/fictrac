/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CameraRemap.h
/// \brief      Base class for remapping images.
/// \author     Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#pragma once


#include "SharedPointers.h"

#include <opencv2/opencv.hpp>

#include <cstdio>

SHARED_PTR(CameraModel);


///
/// Base class for general remapping of one image into another.
///
class Remapper
{
public:
	enum InterpMode {
		NEAREST,
		LINEAR, /// default
		CUBIC
	};
	void setInterpMode(InterpMode mode) { _mode = mode; }

	int getSrcW() { return _srcW; }
	int getSrcH() { return _srcH; }
	int getDstW() { return _dstW; }
	int getDstH() { return _dstH; }

	virtual void apply(const cv::Mat& src, cv::Mat& dst);
	void apply(const IplImage *src, IplImage *dst);

	void applyC1(const unsigned char *src, unsigned char *dst,
			int srcStep=0, int dstStep=0);
	void applyC3(const unsigned char *src, unsigned char *dst,
			int srcStep=0, int dstStep=0);
	void applyC4(const unsigned char *src, unsigned char *dst,
			int srcStep=0, int dstStep=0);
	void applyC1(const char *src, char *dst,
			int srcStep=0, int dstStep=0);
	void applyC3(const char *src, char *dst,
			int srcStep=0, int dstStep=0);
	void applyC4(const char *src, char *dst,
			int srcStep=0, int dstStep=0);

	void applyS1(const unsigned short *src, unsigned short *dst,
			int srcStep=0, int dstStep=0);
	void applyS3(const unsigned short *src, unsigned short *dst,
			int srcStep=0, int dstStep=0);
	void applyS4(const unsigned short *src, unsigned short *dst,
			int srcStep=0, int dstStep=0);

	void applyF1(const float *src, float *dst,
			int srcStep=0, int dstStep=0);
	void applyF3(const float *src, float *dst,
			int srcStep=0, int dstStep=0);
	void applyF4(const float *src, float *dst,
			int srcStep=0, int dstStep=0);

	///
	/// Has become necessary - dodgy ability to modify the map!
	///
	float * getMapX() { return _getMapX(); }
	float * getMapY() { return _getMapY(); }


protected:
	Remapper(int srcW, int srcH, int dstW, int dstH);
	virtual ~Remapper() {}

	enum RemapDataType {
		REMAP_TYPE_C1=(1<<8 | 1),
		REMAP_TYPE_C3=(3<<8 | 3),
		REMAP_TYPE_C4=(4<<8 | 4),
		REMAP_TYPE_S1=(1<<8 | 2),
		REMAP_TYPE_S3=(3<<8 | 6),
		REMAP_TYPE_S4=(4<<8 | 8),
		REMAP_TYPE_F1=(1<<8 | 4),
		REMAP_TYPE_F3=(3<<8 | 12),
		REMAP_TYPE_F4=(4<<8 | 16)
	};
	int _getNumChannels(RemapDataType type)   { return type >> 8; }
	int _getBytesPerPixel(RemapDataType type) { return type & 0xFF; }

	///
	/// Implement to provide the pre-calculated mapping, size = dstW*dstH.
	///
	virtual float * _getMapX() = 0;
	virtual float * _getMapY() = 0;
	static const int INVALID_MAP_VAL = -1; /// set by subclasses if invalid
	static const int INPAINT_MAP_VAL = -2; /// used by inpaint

	///
	/// Note that all apply*() methods go through this virtual _apply()
	/// method, allowing subclasses to change the behaviour if they wish
	/// e.g. BayerRemap.
	///
	virtual void _apply(RemapDataType type,
			const void *src, void *dst,
			int srcStep, int dstStep);

	InterpMode _mode;
	int _srcW, _srcH;
	int _dstW, _dstH;
};
