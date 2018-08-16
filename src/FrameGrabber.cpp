/// FicTrac http://rjdmoore.net/fictrac/
/// \file       FrameGrabber.cpp
/// \brief      Grabs frames from source, performs preprocessing, and adds to frame queue.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//TODO: add colour thresholding

#include "FrameGrabber.h"

#include "Logger.h"
#include "misc.h"

#include <cmath>    // round

using cv::Mat;
using std::shared_ptr;
using std::unique_lock;
using std::mutex;

///
///
///
FrameGrabber::FrameGrabber( shared_ptr<FrameSource> source,
                            CameraRemapPtr          remapper,
                            const Mat&              remap_mask,
                            double                  thresh_ratio,
                            double                  thresh_win_pc,
                            int                     max_buf_len,
                            int                     max_frame_cnt
)   : _source(source), _remapper(remapper), _remap_mask(remap_mask), _active(false)
{
    /// Quick sizes.
    _w = _remapper->getSrcW();
    _h = _remapper->getSrcH();
    _rw = _remapper->getDstW();
    _rh = _remapper->getDstH();

    /// Thresholding.
    if (thresh_ratio < 0) {
        LOG_WRN("Invalid thresh_ratio parameter (%f)! Defaulting to 1.0", thresh_ratio);
        thresh_ratio = 1.0;
    }
    _thresh_ratio = thresh_ratio;

    if ((thresh_win_pc < 0) || (thresh_win_pc > 1.0)) {
        LOG_WRN("Invalid thresh_win parameter (%f)! Defaulting to 0.2", thresh_win_pc);
        thresh_win_pc = 0.2;
    }
    _thresh_win = static_cast<int>(round(thresh_win_pc*_rw)) | 0x01;
    _thresh_rad = static_cast<int>((_thresh_win - 1) / 2);

    LOG_DBG("Thresholding window size: %d (ROI: %d x %d)", _thresh_win, _rw, _rh);

    _max_buf_len = max_buf_len;
    _max_frame_cnt = max_frame_cnt;

    /// Thread stuff.
    _active = true;
    _thread = std::unique_ptr<std::thread>(new std::thread(&FrameGrabber::process, this));
}

///
///
///
FrameGrabber::~FrameGrabber()
{
    LOG("Closing input stream..");

    unique_lock<mutex> l(_qMutex);
    _active = false;
    _qCond.notify_all();
    l.unlock();

    if (_thread && _thread->joinable()) {
        _thread->join();
    }
}

///
///
///
bool FrameGrabber::getFrameSet(Mat& frame, Mat& remap, double& timestamp, bool latest=true)
{
    unique_lock<mutex> l(_qMutex);
    while (_active && (_frame_q.size() == 0)) {
        _qCond.wait(l);
    }
    size_t n = _frame_q.size();
    if (!_active && (n == 0)) {   // n test allows us to finish processing the queue before quitting
        LOG_DBG("No more processed frames in queue!");

        // shouldn't be needed - but just in case :-)
        _qCond.notify_all();

        // mutex unlocked in unique_lock dstr
        return false;
    }

    if ((n != _remap_q.size()) || (n != _ts_q.size())) {
        LOG_ERR("Error! Input processed frame queues are misaligned!");
        
        // drop all frames
        _frame_q.clear();
        _remap_q.clear();
        _ts_q.clear();

        // wake processing thread
        _qCond.notify_all();

        // mutex unlocked in unique_lock dstr
        return false;
    }

    if (latest) {
        frame = _frame_q.back();
        remap = _remap_q.back();
        timestamp = _ts_q.back();
        
        if (n > 1) {
            LOG_WRN("Warning! Dropping %d frame/s from input processed frame queues!", n - 1);
        }

        // drop unused frames
        _frame_q.clear();
        _remap_q.clear();
        _ts_q.clear();
    }
    else {
        frame = _frame_q.front();
        remap = _remap_q.front();
        timestamp = _ts_q.front();

        _frame_q.pop_front();
        _remap_q.pop_front();
        _ts_q.pop_front();

        if (n > 1) {
            LOG_DBG("%d frames remaining in processed frame queue.", _frame_q.size());
        }
    }

    // wake processing thread
    _qCond.notify_all();

    // mutex unlocked in unique_lock dstr
    return true;
}

///
///
///
void FrameGrabber::terminate()
{
    unique_lock<mutex> l(_qMutex);
    _active = false;
    _qCond.notify_all();
}

///
///
///
void FrameGrabber::process()
{
    /// Init storage arrays
    Mat frame_grey(_h, _w, CV_8UC1);
    frame_grey.setTo(cv::Scalar::all(0));
    Mat remap_bgr(_rh, _rw, CV_8UC3);
    remap_bgr.setTo(cv::Scalar::all(0));
    Mat remap_blur(_rh, _rw, CV_8UC1);
    remap_blur.setTo(cv::Scalar::all(128));
    Mat thresh_min(_rh, _rw, CV_8UC1);
    thresh_min.setTo(cv::Scalar::all(0));
    Mat thresh_max(_rh, _rw, CV_8UC1);
    thresh_max.setTo(cv::Scalar::all(0));

    std::unique_ptr<uint8_t[]> win_max_hist = std::unique_ptr<uint8_t[]>(new uint8_t[_thresh_win]);
    std::unique_ptr<uint8_t[]> win_min_hist = std::unique_ptr<uint8_t[]>(new uint8_t[_thresh_win]);

    /// Rewind to video start.
    _source->rewind();

    LOG_DBG("Starting frame grabbing loop!");

    /// Set thread high priority (when run as SU).
    if (!SetThreadVeryHighPriority()) {
        LOG_ERR("Error! Unable to set thread priority!");
    } else {
        LOG("Set frame grabbing thread priority to HIGH!");
    }

    /// Frame grab loop.
    int cnt = 0;
    while (_active) {
        /// Wait until we need to capture a new frame.
        unique_lock<mutex> l(_qMutex);
        while (_active && (_max_buf_len >= 0) && (_frame_q.size() >= _max_buf_len)) {
            _qCond.wait(l);
        }
        l.unlock();
        if (!_active) { break; }

        /// Capture new frame.
        Mat frame_bgr(_h, _w, CV_8UC3);
        frame_bgr.setTo(cv::Scalar::all(0));
        if (!_source->grab(frame_bgr) || ((_max_frame_cnt > 0) && (++cnt > _max_frame_cnt))) {
            if ((_max_frame_cnt > 0) && (++cnt > _max_frame_cnt)) {
                LOG("Max frame count (%d) reached!", _max_frame_cnt);
            } else {
                LOG_ERR("Error grabbing new frame!");
            }
            _active = false;
            l.lock();   // predicate check and wait are not atomic in other thread, so if we don't lock before notifying, notification could be dropped.
            _qCond.notify_all();
            l.unlock();
            break;
        }
        double timestamp = _source->getTimestamp();

        /// Create output remap image in the loop.
        Mat remap_grey(_rh, _rw, CV_8UC1);
        remap_grey.setTo(cv::Scalar::all(128));

        /// Vars for cached min/max.
        int win_it = 0;
        memset(win_max_hist.get(), 0, _thresh_win);
        memset(win_min_hist.get(), 0, _thresh_win);

        /// Create grey ROI frame.
        cv::cvtColor(frame_bgr, frame_grey, CV_BGR2GRAY);
        _remapper->apply(frame_grey, remap_grey);

        /// Blur image before calculating region min/max values.
        medianBlur(remap_grey, remap_blur, 3);

        thresh_min.setTo(cv::Scalar::all(255));
        thresh_max.setTo(cv::Scalar::all(0));

        /** cached min/max **/
        // pre-fill first col
        uint8_t max = 0, min = 255;
        for (int i = 0; i < _thresh_rad; i++) {		// last row is computed in next block (before testing)
            max = 0; min = 255;
            const uint8_t* pmask = _remap_mask.ptr(i);
            uint8_t* pgrey = remap_blur.ptr(i);
            for (int j = 0; j <= _thresh_rad; j++) {
                if (pmask[j] < 255) { continue; }
                uint8_t g = pgrey[j];
                if ((g > max) && (g < 255)) { max = g; }	// ignore overexposed regions
                if (g < min) { min = g; }
            }
            win_max_hist[win_it++] = max;
            win_min_hist[win_it++] = min;
        }

        // compute window min/max
        uint8_t* pthrmax = thresh_max.data;
        uint8_t* pthrmin = thresh_min.data;
        for (int j = 0; j < _rw; j++) {
            for (int i = 0; i < _rh; i++) {
                // add row
                max = 0; min = 255;
                if ((i + _thresh_rad) < _rh) {
                    const uint8_t* pmask = _remap_mask.ptr(i + _thresh_rad);
                    uint8_t* pgrey = remap_blur.ptr(i + _thresh_rad);
                    for (int s = -_thresh_rad; s <= _thresh_rad; s++) {
                        int js = j + s;
                        if ((js < 0) || (js >= _rw)) { continue; }
                        if (pmask[js] < 255) { continue; }
                        uint8_t g = pgrey[js];
                        if ((g > max) && (g < 255)) { max = g; }	// ignore overexposed regions
                        if (g < min) { min = g; }
                    }
                }
                else {
                    // pre-fill next cols
                    const uint8_t* pmask = _remap_mask.ptr(i + _thresh_rad - _rh);
                    uint8_t* pgrey = remap_blur.ptr(i + _thresh_rad - _rh);
                    for (int s = -_thresh_rad; s <= _thresh_rad; s++) {
                        int js = j + s + 1;
                        if ((js < 0) || (js >= _rw)) { continue; }
                        if (pmask[js] < 255) { continue; }
                        uint8_t g = pgrey[js];
                        if ((g > max) && (g < 255)) { max = g; }	// ignore overexposed regions
                        if (g < min) { min = g; }
                    }
                }
                win_max_hist[win_it] = max;
                win_min_hist[win_it] = min;

                // find window max/min
                max = 0; min = 255;
                for (int k = 0; k < _thresh_win; k++) {
                    int ik = i + _thresh_rad - k;
                    if ((ik >= _rh) || (ik < 0)) { continue; }
                    int wk = win_it - k;
                    if (wk < 0) { wk += _thresh_win; }
                    uint8_t mx = win_max_hist[wk];
                    if (mx > max) { max = mx; }
                    uint8_t mn = win_min_hist[wk];
                    if (mn < min) { min = mn; }
                }
                pthrmax[i*thresh_max.step + j] = max;
                pthrmin[i*thresh_min.step + j] = min;
                if (++win_it >= _thresh_win) { win_it -= _thresh_win; }
            }
        }

        // apply thresholding
        for (int i = 0; i < _rh; i++) {
            const uint8_t* pmask = _remap_mask.ptr(i);
            uint8_t* premap = remap_grey.ptr(i);
            uint8_t* pthrmin = thresh_min.ptr(i);
            uint8_t* pthrmax = thresh_max.ptr(i);
            for (int j = 0; j < _rw; j++) {
                if (pmask[j] != 255) {
                    premap[j] = 128;
                    continue;
                }
                if ((_thresh_ratio*(premap[j] - pthrmin[j])) <= (pthrmax[j] - premap[j])) {
                    premap[j] = 0;
                }
                else {
                    premap[j] = 255;
                }
            }
        }

        /// Re-obtain lock and add processed frame to queue.
        l.lock();
        _frame_q.push_back(frame_bgr);
        _remap_q.push_back(remap_grey);
        _ts_q.push_back(timestamp);
        _qCond.notify_all();

        LOG_DBG("Processed frame added to input queue (l = %d).", _frame_q.size());

        l.unlock();
    }

    LOG_DBG("Stopping frame grabbing loop!");
}
