/// FicTrac http://rjdmoore.net/fictrac/
/// \file       FrameGrabber.h
/// \brief      Grabs frames from source, performs preprocessing, and adds to frame queue.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "CameraRemap.h"
#include "FrameSource.h"

#include <opencv2/opencv.hpp>

#include <memory>	// shared_ptr, unique_ptr
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

///
/// 
///
class FrameGrabber
{
public:
    FrameGrabber(   std::shared_ptr<CameraRemap>    remapper,
                    std::shared_ptr<FrameSource>    source,
                    double                          thresh_ratio,
                    double                          thresh_win,
                    int                             max_buf_len,
                    cv::Mat&                        remap_mask
    );
    
    bool getFrameSet(cv::Mat& frame, cv::Mat& remap, double& timestamp, bool latest);
    bool getLatestFrameSet(cv::Mat& frame, cv::Mat& remap, double& timestamp) {
        return getFrameSet(frame, remap, timestamp, true);
    }
    bool getNextFrameSet(cv::Mat& frame, cv::Mat& remap, double& timestamp) {
        return getFrameSet(frame, remap, timestamp, false);
    }

private:
    /// Worker function.
    void process();

    std::shared_ptr<CameraRemap> _remapper;
    std::shared_ptr<FrameSource> _source;

    int _w, _h, _rw, _rh;

    cv::Mat _remap_mask;

    double _thresh_ratio;
    int _thresh_win, _thresh_rad;

    int _max_buf_len;

    bool _active;
    std::unique_ptr<std::thread> _thread;
    std::mutex _qMutex;
    std::condition_variable _qCond;

    /// Output queues.
    std::deque<cv::Mat> _frame_q, _remap_q;
    std::deque<double> _ts_q;
};
