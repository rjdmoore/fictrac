/// FicTrac http://rjdmoore.net/fictrac/
/// \file       PGRSource.h
/// \brief      PGR USB2/3 sources (FlyCapture/Spinnaker SDK).
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "FrameSource.h"

#if defined(PGR_USB3)
#include <Spinnaker.h>
#elif defined(PGR_USB2)
#include <FlyCapture2.h>
#include <memory>
#endif // PGR_USB2/3

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
#if defined(PGR_USB3)
    Spinnaker::SystemPtr _system;
    Spinnaker::CameraList _camList;
    Spinnaker::CameraPtr _cam;
#elif defined(PGR_USB2)
    std::shared_ptr<FlyCapture2::Camera> _cam;
#endif // PGR_USB2/3
};
