/// FicTrac http://rjdmoore.net/fictrac/
/// \file       PGRSource.h
/// \brief      PGR (FlyCapture) sources.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#ifdef PGR_CAMERA

#include "ImgSource.h"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <flycapture/FlyCapture2.h>

class PGRSource : public ImgSource {
public:
	PGRSource(int index=0);
	virtual ~PGRSource();

	virtual bool setFPS(int fps);
	virtual bool rewind();
//	virtual void skip(unsigned int frames);
	virtual bool grab(cv::Mat& frame);

private:
	boost::shared_ptr<FlyCapture2::Camera> _cap;
	FlyCapture2::Image _frame_cap;
};

#endif
