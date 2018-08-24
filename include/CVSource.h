/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CVSource.h
/// \brief      OpenCV frame sources.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "FrameSource.h"

#include <opencv2/opencv.hpp>
/// OpenCV individual includes required by gcc?
#include <opencv2/videoio.hpp>

#include <memory>	// shared_ptr
#include <cstdio>

class CVSource : public FrameSource {
public:
	CVSource(std::string input);
	virtual ~CVSource();

    virtual double getFPS();
	virtual bool setFPS(double fps);
	virtual bool rewind();
	virtual bool grab(cv::Mat& frame);

private:
	std::shared_ptr<cv::VideoCapture> _cap;
	cv::Mat _frame_cap;
};
