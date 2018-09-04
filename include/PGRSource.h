/// FicTrac http://rjdmoore.net/fictrac/
/// \file       PGRSource.h
/// \brief      PGR USB3 sources (Spinnaker SDK).
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#ifdef PGR_USB3

#include "FrameSource.h"

#include <Spinnaker.h>
#include <opencv2/opencv.hpp>

class PGRSource : public FrameSource {
public:
	PGRSource(int index=0);
	virtual ~PGRSource();

    virtual double getFPS();
	virtual bool setFPS(double fps);
    virtual bool rewind() { return false; };
	virtual bool grab(cv::Mat& frame);

private:
    Spinnaker::SystemPtr _system;
    Spinnaker::CameraList _camList;
    Spinnaker::CameraPtr _cam;
};

#endif
