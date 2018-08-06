/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CVSource.cpp
/// \brief      OpenCV frame sources.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "CVSource.h"

#include "Logger.h"
#include "timing.h"


///
/// Constructor for camera input.
///
CVSource::CVSource(int index)
{
	LOG("Looking for camera at index %d ...", index);

	_cap = std::shared_ptr<cv::VideoCapture>(new cv::VideoCapture(index));
	_open = _cap->isOpened();

	if( _open ) {
		_width = static_cast<int>(_cap->get(CV_CAP_PROP_FRAME_WIDTH));
		_height = static_cast<int>(_cap->get(CV_CAP_PROP_FRAME_HEIGHT));
	}

    _live = true;
}

///
/// Constructor for video file input.
///
CVSource::CVSource(std::string filename)
{
	LOG("Looking for input video file %s ...", filename.c_str());

	_cap = std::shared_ptr<cv::VideoCapture>(new cv::VideoCapture(filename.c_str()));
	_open = _cap->isOpened();

	if( _open ) {
		_width = static_cast<int>(_cap->get(CV_CAP_PROP_FRAME_WIDTH));
		_height = static_cast<int>(_cap->get(CV_CAP_PROP_FRAME_HEIGHT));
	}

    _live = false;
}

///
/// Default destructor.
///
CVSource::~CVSource()
{}

///
/// Set input source fps.
///
bool CVSource::setFPS(int fps)
{
    bool ret = false;
	if( _open && (fps > 0) ) {
        if (!_cap->set(CV_CAP_PROP_FPS, fps)) {
            LOG_WRN("Warning! Failed to set source fps (attempted to set fps=%d).", fps);
        } else { ret = true; }
    }
    return ret;
}

///
/// Rewind input source to beginning.
/// Ignored by non-file sources.
///
bool CVSource::rewind()
{
    bool ret = false;
	if (_open) {
        if (!_cap->set(CV_CAP_PROP_POS_FRAMES, 0)) {
            LOG_WRN("Warning! Failed to rewind source.");
        } else { ret = true; }
	}
    return ret;
}

///
/// Capture and retrieve frame from source.
///
bool CVSource::grab(cv::Mat& frame)
{
	if( !_open ) { return false; }
	if( !_cap->read(_frame_cap) ) {
		LOG_ERR("Error grabbing image frame!");
		return false;
	}
    double ts = static_cast<double>(ts_ms());    // backup, in case the device timestamp is junk
	_timestamp = _cap->get(CV_CAP_PROP_POS_MSEC);
    if (_timestamp <= 0) {
        _timestamp = ts;
    }
	if( _frame_cap.channels() == 1 ) {
		switch( _bayerType ) {
			case BAYER_BGGR:
				cv::cvtColor(_frame_cap, frame, CV_BayerBG2BGR);
				break;
			case BAYER_GBRG:
				cv::cvtColor(_frame_cap, frame, CV_BayerGB2BGR);
				break;
			case BAYER_GRBG:
				cv::cvtColor(_frame_cap, frame, CV_BayerGR2BGR);
				break;
			case BAYER_RGGB:
				cv::cvtColor(_frame_cap, frame, CV_BayerRG2BGR);
				break;
			case BAYER_NONE:
			default:
				cv::cvtColor(_frame_cap, frame, CV_GRAY2BGR);
				break;
		}
	} else {
		_frame_cap.copyTo(frame);
	}
	return true;
}
