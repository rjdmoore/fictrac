/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CVSource.cpp
/// \brief      OpenCV frame sources.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "CVSource.h"

#include "Logger.h"


///
/// Constructor for camera input.
///
CVSource::CVSource(int index)
{
	LOG("Looking for camera at index %d ...", index);

	_cap = std::shared_ptr<cv::VideoCapture>(new cv::VideoCapture(index));
	_open = _cap->isOpened();

	if( _open ) {
		_width = _cap->get(CV_CAP_PROP_FRAME_WIDTH);
		_height = _cap->get(CV_CAP_PROP_FRAME_HEIGHT);
	}
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
		_width = _cap->get(CV_CAP_PROP_FRAME_WIDTH);
		_height = _cap->get(CV_CAP_PROP_FRAME_HEIGHT);
	}
}

///
/// Default destructor.
///
CVSource::~CVSource()
{}

///
/// Set input source fps.
///
void CVSource::setFPS(int fps)
{
	if( _open && (fps > 0) ) {
		_cap->set(CV_CAP_PROP_FPS, fps);
	}
}

///
/// Rewind input source to beginning.
/// Ignored by non-file sources.
///
void CVSource::rewind()
{
	if (_open) {
		_cap->set(CV_CAP_PROP_POS_FRAMES, 0);
	}
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
	_timestamp = _cap->get(CV_CAP_PROP_POS_MSEC);
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
