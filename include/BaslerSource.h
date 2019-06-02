#pragma once

#include "FrameSource.h"

// Include Basler Pylon libraries
#include <pylon/PylonIncludes.h>
#include <pylon/ImageFormatConverter.h>
#include <pylon/usb/BaslerUsbInstantCameraArray.h>
#ifdef PYLON_WIN_BUILD
#	include <pylon/PylonGUI.h>
#endif


class BaslerSource : public FrameSource {

public:
  BaslerSource(int index=0);
  virtual ~BaslerSource();

  virtual double getFPS();
  virtual bool setFPS(double);

  virtual bool rewind() {return false; };
  virtual bool grab(cv::Mat& frame);

private:
  Pylon::CPylonImage pylonImg;
  Pylon::CGrabResultPtr ptrGrabResult;
  Pylon::CInstantCamera cam;

};
