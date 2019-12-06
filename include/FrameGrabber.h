/// FicTrac http://rjdmoore.net/fictrac/
/// \file       FrameGrabber.h
/// \brief      Grabs frames from source, performs preprocessing, and adds to frame queue.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "CameraModel.h"
#include "CameraRemap.h"
#include "FrameSource.h"

#include <opencv2/opencv.hpp>

#include <memory>	// shared_ptr, unique_ptr
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <deque>

///
/// 
///
class FrameGrabber
{
public:
    FrameGrabber(   std::shared_ptr<FrameSource>    source,
                    CameraRemapPtr                  remapper,
                    const cv::Mat&                  remap_mask,
                    double                          thresh_ratio,
                    double                          thresh_win_pc,
                    std::string                     thresh_rgb_transform = "grey",
                    int                             max_buf_len = 1,
                    int                             max_frame_cnt = -1
    );
    ~FrameGrabber();

    void terminate();
    
    bool getFrameSet(cv::Mat& frame, cv::Mat& remap, double& timestamp, double& ms_since_midnight, bool latest = true);
    bool getLatestFrameSet(cv::Mat& frame, cv::Mat& remap, double& timestamp, double& ms_since_midnight) {
        return getFrameSet(frame, remap, timestamp, ms_since_midnight, true);
    }
    bool getNextFrameSet(cv::Mat& frame, cv::Mat& remap, double& timestamp, double& ms_since_midnight) {
        return getFrameSet(frame, remap, timestamp, ms_since_midnight, false);
    }

private:
    /// Worker function.
    void process();

    std::shared_ptr<FrameSource> _source;
    CameraRemapPtr _remapper;

    int _w, _h, _rw, _rh;

    const cv::Mat _remap_mask;

    double _thresh_ratio;
    int _thresh_win, _thresh_rad;
    enum {
        GREY,
        RED,
        GREEN,
        BLUE
    } _thresh_rgb_transform;

    int _max_buf_len, _max_frame_cnt;

    /// Thread stuff.
    std::atomic_bool _active;
    std::unique_ptr<std::thread> _thread;
    std::mutex _qMutex;
    std::condition_variable _qCond;

    /// Output queues.
    std::deque<cv::Mat> _frame_q, _remap_q;
    std::deque<double> _ts_q, _ms_q;
};
