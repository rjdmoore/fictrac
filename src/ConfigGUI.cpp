/// FicTrac http://rjdmoore.net/fictrac/
/// \file       configGui.cpp
/// \brief      Interactive GUI for configuring FicTrac.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//TODO: check config.is_open()
//TODO: Add support for fisheye camera model.

#include "ConfigGui.h"

#include "typesvars.h"
#include "CameraModel.h"
#include "geometry.h"
#include "drawing.h"
#include "logging.h"

#include <vector>   // vector
#include <cassert>  // assert

using cv::Mat;
using cv::Point2d;
using std::vector;
using std::string;

///
/// Constant variables.
///
const int       ZOOM_DIM    = 600;
const double    ZOOM_SCL    = 1.0 / 10.0;

const int NCOLOURS = 6;
cv::Scalar COLOURS[NCOLOURS] = {
    CV_RGB(255, 0,   0),
    CV_RGB(0,   255, 0),
    CV_RGB(0,   0,   255),
    CV_RGB(255, 255, 0),
    CV_RGB(0,   255, 255),
    CV_RGB(255, 0,   255)
};

///
/// Collect mouse events from config GUI window.
///
void onMouseEvent(int event, int x, int y, int f, void* ptr)
{
    ConfigGui::INPUT_DATA* pdata = static_cast<ConfigGui::INPUT_DATA*>(ptr);
    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
            break;
            
        case CV_EVENT_LBUTTONUP:
            switch(pdata->mode)
            {
                case ConfigGui::CIRC_PTS:
                    pdata->circPts.push_back(Point2d(x,y));
                    break;
                case ConfigGui::IGNR_PTS:
                    // ensure there is at least one active ignore region
                    if (pdata->ignrPts.empty()) { pdata->ignrPts.push_back(vector<Point2d>()); }
                    // add click to the active ignore region
                    pdata->ignrPts.back().push_back(Point2d(x,y)); 
                    break;
                default:
                    break;
            }
            break;
        
        case CV_EVENT_RBUTTONUP:
            switch(pdata->mode)
            {
                case ConfigGui::CIRC_PTS:
                    if (pdata->circPts.size() > 0) { pdata->circPts.pop_back(); }
                    break;
                case ConfigGui::IGNR_PTS:
                    if (!pdata->ignrPts.empty()) {
                        // if the active ignore region is empty, remove it
                        if (pdata->ignrPts.back().empty()) { pdata->ignrPts.pop_back(); }
                        // otherwise remove points from the active ignore region
                        else { pdata->ignrPts.back().pop_back(); }
                    }
                    break;
                default:
                    break;
            }
            break;
        
        case CV_EVENT_MOUSEMOVE:
            pdata->cursorPt.x = x;
            pdata->cursorPt.y = y;
            break;

        default:
            break;
    }
}

///
/// Create a zoomed ROI.
///
void createZoomROI(Mat& zoom_roi, const Mat& frame, const Point2d& pt, int orig_dim)
{
    int x = frame.cols/2;
    if (pt.x >= 0) { x = clamp(int(pt.x - orig_dim/2 + 0.5), int(orig_dim/2), frame.cols - 1 - orig_dim); }
    int y = frame.rows/2;
    if (pt.y >= 0) { y = clamp(int(pt.y - orig_dim/2 + 0.5), int(orig_dim/2), frame.rows - 1 - orig_dim); }
    Mat crop_rect = frame(cv::Rect(x, y, orig_dim, orig_dim));
    cv::resize(crop_rect, zoom_roi, zoom_roi.size());
}

///
/// Constructor.
///
ConfigGui::ConfigGui(string config_fn)
: _config_fn(config_fn)
{
    /// Load and parse config file.
    _valid = (_cfg.read(_config_fn) > 0);
}

///
/// Destructor.
///
ConfigGui::~ConfigGui()
{}

///
/// Prepare input image for user input.
///
bool ConfigGui::setFrame(Mat& frame)
{
    assert(frame.channels() == 1 || frame.channels() == 3);
    
    /// Copy input frame.
    if (frame.channels() == 3) {
        cv::cvtColor(frame, _frame, CV_BGR2GRAY);
    } else if (frame.channels() == 1) {
        _frame = frame.clone();
    } else {
        // uh oh, shouldn't get here
        BOOST_LOG_TRIVIAL(error) << "Unexpected number of image channels (" << frame.channels() << ")!";
        return (_valid = false);
    }
    
    /// Stretch contrast for display
    histStretch(_frame);
    assert(!_frame.empty());
    _w = _frame.cols;
    _h = _frame.rows;
    
    /// Load camera model.
    double vfov = 0;
    _cfg.getDbl("vfov", vfov);
    assert(vfov>0);
    BOOST_LOG_TRIVIAL(info) << "Using vfov: " << vfov << " degrees";
    
    //TODO: Add support for fisheye camera model.
    _cam_model = CameraModel::createRectilinear(_w, _h, vfov * CM_D2R);
    
    return _valid;
}

///
/// Run user input program or configuration.
///
bool ConfigGui::run()
{
    if (!_valid) { return _valid; }
    
    /// Interactive window.
    cv::namedWindow("configGUI", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("configGUI", onMouseEvent, &_input_data);
    
    /// Display/input loop.
    CmPoint c;
    double r = -1;
    char key = 0;
    vector<int> pts_to_dump;
    vector<vector<int> > polys_to_dump;
    BOOST_LOG_TRIVIAL(debug) << "New state: INIT";
    _input_data.mode = INIT;
    const char exit_key = 0x1b;
    const char enter_key = 0x0a;
    const int click_rad = std::max(int(_w/150+0.5), 5);
    Mat disp_frame, zoom_frame(ZOOM_DIM, ZOOM_DIM, CV_8UC1);
    while (key != exit_key) {
        /// Create frame for drawing.
        cv::cvtColor(_frame, disp_frame, CV_GRAY2RGB);
        
        switch (_input_data.mode)
        {
            /// Display instruction text.
            case INIT:
                printf("\n  Define the circumference of the track ball.\n\n  Use the left mouse button to add new points.\n  You must select at least 3 (but preferably 6+) points around the circumference of the track ball.\n  NOTE! Be careful to place points only on the circumference of the track ball,\nand not along the outline of the visible track ball where the actual circumference has been partially obscured.\n  You can use the right mouse button to remove the last added point.\n  The fitted circumference is drawn in red.\n\n  Press ENTER when you are satisfied with the fitted circumference, or press ESC to exit...\n\n");
                BOOST_LOG_TRIVIAL(debug) << "New state: CIRC_PTSs";
                _input_data.mode = CIRC_PTS;
                break;
            
            /// Input circumference points.
            case CIRC_PTS:
                /// Fit circular FoV to sphere.
                if (_input_data.circPts.size() >= 3) {
                    circleFit_camModel(_input_data.circPts, _cam_model, c, r);
                }
                
                /// Draw previous clicks.
                for (auto click : _input_data.circPts) {
                    cv::circle(disp_frame, click, click_rad, CV_RGB(255,255,0), 1, CV_AA);
                }
                
                /// State machine logic.
                if (key == enter_key) {
                    if (_input_data.circPts.size() >= 3) {
                        // dump circumference points to config file
                        pts_to_dump.clear();
                        for (auto p : _input_data.circPts) {
                            pts_to_dump.push_back(p.x);
                            pts_to_dump.push_back(p.y);
                        }
                        
                        // write to config file
                        BOOST_LOG_TRIVIAL(info) << "Adding roi_circ to config file and writing to disk (" << _config_fn << ")...";
                        _cfg.add("roi_circ", pts_to_dump);
                        assert(_cfg.write() > 0);
                        
                        // test read
                        BOOST_LOG_TRIVIAL(debug) << "Re-loading config file and reading roi_circ...";
                        _cfg.read(_config_fn);
                        assert(_cfg.getVecInt("roi_circ", pts_to_dump));
                        
                        // advance state
                        printf("\n  Define ignore regions.\n\n  Use the left mouse button to add points to a new polygon.\n  Polygons can be drawn around objects (such as the animal) that block the view of the track ball.\n  You can use the right mouse button to remove the last added point.\n\n  Press ENTER to start a new polygon, or press ENTER twice when you are satisfied with the selected ignore regions, or press ESC to exit...\n\n");
                        BOOST_LOG_TRIVIAL(debug) << "New state: IGNR_PTS";
                        _input_data.mode = IGNR_PTS;
                    } else {
                        BOOST_LOG_TRIVIAL(warning) << "You must select at least 3 circumference points (you have selected " << _input_data.circPts.size() << " pts)!";
                    }
                }
                break;
            
            /// Input ignore regions.
            case IGNR_PTS:
                /// Draw previous clicks.
                for (unsigned int i = 0; i < _input_data.ignrPts.size(); i++) {
                    for (unsigned int j = 0; j < _input_data.ignrPts[i].size(); j++) {
                        if (i == _input_data.ignrPts.size()-1) {
                            cv::circle(disp_frame, _input_data.ignrPts[i][j], click_rad, COLOURS[i%NCOLOURS], 1, CV_AA);
                        }
                        cv::line(disp_frame, _input_data.ignrPts[i][j], _input_data.ignrPts[i][(j+1)%_input_data.ignrPts[i].size()], COLOURS[i%NCOLOURS], 1, CV_AA);
                    }
                }
                
                /// State machine logic.
                if (key == enter_key) {
                    // if current poly is empty, assume we've finished
                    if (_input_data.ignrPts.back().empty()) {
                        _input_data.ignrPts.pop_back();
                        
                        // dump ignore region polys to config file
                        polys_to_dump.clear();
                        for (auto poly : _input_data.ignrPts) {
                            polys_to_dump.push_back(vector<int>());
                            for (auto pt : poly) {
                                polys_to_dump.back().push_back(pt.x);
                                polys_to_dump.back().push_back(pt.y);
                            }
                        }
                        
                        // write to config file
                        BOOST_LOG_TRIVIAL(info) << "Adding roi_ignr to config file and writing to disk (" << _config_fn << ")...";
                        _cfg.add("roi_ignr", polys_to_dump);
                        assert(_cfg.write() > 0);
                        
                        // test read
                        BOOST_LOG_TRIVIAL(debug) << "Re-loading config file and reading roi_ignr...";
                        _cfg.read(_config_fn);
                        assert(_cfg.getVVecInt("roi_ignr", polys_to_dump));
                        
                        // advance state
                        BOOST_LOG_TRIVIAL(debug) << "New state: EXIT";
                        _input_data.mode = EXIT;
                    }
                    // otherwise, start a new poly
                    else {
                        _input_data.addPoly();
                        BOOST_LOG_TRIVIAL(info) << "New ignore region added!";
                    }
                }
                break;
            
            //TODO: Add state for defining animal coordinate frame.
            
            /// Exit config.
            case EXIT:
                key = exit_key;
            default:
                break;
        }
        if (key == exit_key) { break; }
        
        /// Draw fitted circumference.
        if (r > 0) { drawCircle_camModel(disp_frame, _cam_model, c, r, CV_RGB(255,0,0), false); }
        
        /// Draw cursor location.
        drawCursor(disp_frame, _input_data.cursorPt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS]);
        
        /// Create zoomed window.
        const int scaled_zoom_dim = ZOOM_DIM * ZOOM_SCL;
        createZoomROI(zoom_frame, disp_frame, _input_data.cursorPt, scaled_zoom_dim);
        
        /// Display.
        cv::imshow("zoomROI", zoom_frame);
        cv::imshow("configGUI", disp_frame);
        key = cv::waitKey(5);
    }
    
    BOOST_LOG_TRIVIAL(info) << "Exiting configGui!";
    
    cv::destroyAllWindows();
    return _valid;
}