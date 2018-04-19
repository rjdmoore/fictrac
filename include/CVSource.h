/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CVSource.h
/// \brief      OpenCV frame sources.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "FrameSource.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <boost/shared_ptr.hpp>

#include <cstdio>

class CVSource : public FrameSource {
public:
	CVSource(int index=0);
	CVSource(std::string filename);
	virtual ~CVSource();

	virtual void setFPS(int fps);
	virtual void rewind();
	virtual bool grab(cv::Mat& frame);

private:
	boost::shared_ptr<cv::VideoCapture> _cap;
	cv::Mat _frame_cap;
};
