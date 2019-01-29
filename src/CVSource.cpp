/// FicTrac http://rjdmoore.net/fictrac/
/// \file       CVSource.cpp
/// \brief      OpenCV frame sources.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "CVSource.h"

#include "Logger.h"
#include "timing.h"

/// OpenCV individual includes required by gcc?
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <exception>

using cv::Mat;


///
/// Constructor.
///
CVSource::CVSource(std::string input)
{
    LOG_DBG("Source is: %s", input.c_str());
    Mat test_frame;
    try {
        // try reading input as camera id
        LOG_DBG("Trying source as camera id..");
        int id = std::stoi(input);
        _cap = std::shared_ptr<cv::VideoCapture>(new cv::VideoCapture(id));
        if (!_cap->isOpened()) { throw 0; }
        *_cap >> test_frame;
        if (test_frame.empty()) { throw 0; }
        LOG("Using source type: camera id.");
        _open = true;
        _live = true;
    }
    catch (...) {
        try {
            // then try loading as video file
            LOG_DBG("Trying source as video file..");
            _cap = std::shared_ptr<cv::VideoCapture>(new cv::VideoCapture(input));
            if (!_cap->isOpened()) { throw 0; }
            *_cap >> test_frame;
            if (test_frame.empty()) { throw 0; }
            LOG("Using source type: video file.");
            _open = true;
            _live = false;
        }
        catch (...) {
            LOG_ERR("Could not interpret source type (%s)!", input.c_str());
            _open = false;
        }
    }

	if( _open ) {
		_width = static_cast<int>(_cap->get(cv::CAP_PROP_FRAME_WIDTH));
		_height = static_cast<int>(_cap->get(cv::CAP_PROP_FRAME_HEIGHT));
        if (_live) {
            _fps = getFPS();    // don't init fps for video files - we might want to play them back as fast as possible

            LOG("OpenCV camera initialised (%dx%d @ %.3f fps)!", _width, _height, _fps);
        }
        else {
            LOG("OpenCV video initialised (%dx%d)!", _width, _height);
        }
	}
}

///
/// Default destructor.
///
CVSource::~CVSource()
{}

///
/// Get input source fps.
///
double CVSource::getFPS()
{
    double fps = _fps;
    if (_open) {
        fps = _cap->get(cv::CAP_PROP_FPS);
    }
    return fps;
}

///
/// Set input source fps.
///
bool CVSource::setFPS(double fps)
{
    bool ret = false;
    if (_open && (fps > 0)) {
        if (!_cap->set(cv::CAP_PROP_FPS, fps)) {
            LOG_WRN("Warning! Failed to set device fps (attempted to set fps=%.2f).", fps);
            _fps = fps; // just set fps anyway for playback
            LOG("Playback frame rate is now %.2f", _fps);
        }
        else {
            _fps = getFPS();
            LOG("Device frame rate is now %.2f", _fps);
        }
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
        if (!_cap->set(cv::CAP_PROP_POS_FRAMES, 0)) {
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
	_timestamp = _cap->get(cv::CAP_PROP_POS_MSEC);
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

    /// Correct average frame rate when reading from file.
    if (!_live && (_fps > 0)) {
        static double prev_ts = ts - (1000/_fps);
        static double av_fps = _fps;      // initially 40 Hz
        static double sleep_ms = 1000/_fps;
        av_fps = 0.15 * av_fps + 0.85 * (1000 / (ts - prev_ts));
        sleep_ms *= 0.25 * (av_fps / _fps) + 0.75;
        sleep(static_cast<long>(round(sleep_ms)));
        prev_ts = ts;
    }

	return true;
}
