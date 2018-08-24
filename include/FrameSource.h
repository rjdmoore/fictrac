/// FicTrac http://rjdmoore.net/fictrac/
/// \file       FrameSource.h
/// \brief      Abstract template for frame sources.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include <opencv2/opencv.hpp>

enum BAYER_TYPE { BAYER_NONE, BAYER_RGGB, BAYER_GRBG, BAYER_GBRG, BAYER_BGGR };

class FrameSource {
public:
	FrameSource() : _open(false), _bayerType(BAYER_NONE), _width(-1), _height(-1), _timestamp(-1), _fps(-1), _live(true) {}
	virtual ~FrameSource() {}

    virtual double getFPS()=0;
	virtual bool setFPS(double fps)=0;
	virtual bool rewind()=0;
	virtual bool grab(cv::Mat& frame)=0;

	bool isOpen() { return _open; }
	int getWidth() { return _width; }
	int getHeight() { return _height; }
	double getTimestamp() { return _timestamp; }
	void setBayerType(BAYER_TYPE bayer_type) { _bayerType = bayer_type; }
    bool isLive() { return _live; }

protected:
	bool _open;
	BAYER_TYPE _bayerType;
	int _width, _height;
	double _timestamp, _fps;
    bool _live;
};
