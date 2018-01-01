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
#include <iostream> // getline, stoi
#include <cstdio>   // getchar
#include <exception>

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
                    
                case ConfigGui::R_XY:
                case ConfigGui::R_YZ:
                case ConfigGui::R_XZ:
                    pdata->sqrePts.push_back(Point2d(x,y));
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
                    
                case ConfigGui::R_XY:
                case ConfigGui::R_YZ:
                case ConfigGui::R_XZ:
                    if (pdata->sqrePts.size() > 0) { pdata->sqrePts.pop_back(); }
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
: _open(false), _config_fn(config_fn)
{
    /// Load and parse config file.
    _open = (_cfg.read(_config_fn) > 0);
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
        return (_open = false);
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
    
    return _open;
}

///
/// Run user input program or configuration.
///
bool ConfigGui::run()
{
    if (!_open) { return _open; }
    
    /// Interactive window.
    cv::namedWindow("configGUI", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("configGUI", onMouseEvent, &_input_data);
    
    /// Display/input loop.
    CmPoint c;
    double r = -1;
    char key = 0;
    double vec[3];
    Point2d pt0, pt;
    vector<int> cfg_pts;
    vector<vector<int> > cfg_polys;
    BOOST_LOG_TRIVIAL(debug) << "New state: INIT";
    _input_data.mode = CIRC_INIT;
    const char exit_key = 0x1b;
    const char enter_key = 0x0a;
    const int click_rad = std::max(int(_w/150+0.5), 5);
    Mat disp_frame, zoom_frame(ZOOM_DIM, ZOOM_DIM, CV_8UC1);
    const int scaled_zoom_dim = ZOOM_DIM * ZOOM_SCL;
    while (key != exit_key) {
        /// Create frame for drawing.
        cv::cvtColor(_frame, disp_frame, CV_GRAY2RGB);
        
        int in;
        string str;
        switch (_input_data.mode)
        {
            /// Check for existing circumference points.
            case CIRC_INIT:
            
                // test read
                cfg_pts.clear();
                if (_cfg.getVecInt("roi_circ", cfg_pts)) {
                    
                    /// Load circumference points from config file.
                    _input_data.circPts.clear();
                    for (unsigned int i = 1; i < cfg_pts.size(); i+=2) {
                        _input_data.circPts.push_back(Point2d(cfg_pts[i-1], cfg_pts[i]));
                    }
                    
                    /// Fit circular FoV to sphere.
                    if (_input_data.circPts.size() >= 3) {
                        circleFit_camModel(_input_data.circPts, _cam_model, c, r);
                        
                        /// Draw fitted circumference.
                        if (r > 0) {
                            drawCircle_camModel(disp_frame, _cam_model, c, r, CV_RGB(255,0,0), false);
                            
                            /// Display.
                            cv::imshow("configGUI", disp_frame);
                            cv::waitKey(100);   //FIXME: why do we have to wait so long to make sure the frame is drawn?
                            
                            printf("\n\n\n  Sphere circumference points were found in the config file. You can discard these points and re-run config or keep the existing points.\n");
                            
                            // input loop
                            while (true) {
                                cv::waitKey(100);   //FIXME: dirty hack - sometimes image doesn't draw, at least with this line we can just mash keys until it does
                                printf("\n  Would you like to keep the existing circumference points ([y]/n)? ");
                                in = std::getchar();
                                switch (in)
                                {
                                    case 'y':
                                    case 'Y':
                                        std::getchar(); // discard \n
                                    case '\n':
                                        // advance state
                                        BOOST_LOG_TRIVIAL(debug) << "New state: IGNR_INIT";
                                        _input_data.mode = IGNR_INIT;
                                        break;
                                    case 'n':
                                    case 'N':
                                        std::getchar(); // discard \n
                                        break;
                                    default:
                                        BOOST_LOG_TRIVIAL(error) << "Invalid input!";
                                        std::getchar(); // discard \n
                                        continue;
                                        break;
                                }
                                break;
                            }
                        }
                    }
                }
                
                if (_input_data.mode == CIRC_INIT) {
                    _input_data.circPts.clear();
                    printf("\n\n\n  Define the circumference of the track ball.\n\n  Use the left mouse button to add new points.\n  You must select at least 3 (but preferably 6+) points around the circumference of the track ball.\n  NOTE! Be careful to place points only on the circumference of the track ball,\nand not along the outline of the visible track ball where the actual circumference has been partially obscured.\n  You can use the right mouse button to remove the last added point.\n  The fitted circumference is drawn in red.\n\n  Press ENTER when you are satisfied with the fitted circumference, or press ESC to exit...\n\n");
                    BOOST_LOG_TRIVIAL(debug) << "New state: CIRC_PTS";
                    _input_data.mode = CIRC_PTS;
                }
                break;
            
            /// Input circumference points.
            case CIRC_PTS:
                /// Fit circular FoV to sphere.
                if (_input_data.circPts.size() >= 3) {
                    circleFit_camModel(_input_data.circPts, _cam_model, c, r);
                } else {
                    r = -1;
                }
                
                /// Draw previous clicks.
                for (auto click : _input_data.circPts) {
                    cv::circle(disp_frame, click, click_rad, CV_RGB(255,255,0), 1, CV_AA);
                }
                
                /// Draw fitted circumference.
                if (r > 0) { drawCircle_camModel(disp_frame, _cam_model, c, r, CV_RGB(255,0,0), false); }
                
                /// Draw cursor location.
                drawCursor(disp_frame, _input_data.cursorPt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS]);
                
                /// Create zoomed window.
                createZoomROI(zoom_frame, disp_frame, _input_data.cursorPt, scaled_zoom_dim);
                
                /// Display.
                cv::imshow("zoomROI", zoom_frame);
                cv::imshow("configGUI", disp_frame);
                key = cv::waitKey(5);
                
                /// State machine logic.
                if (key == enter_key) {
                    if (_input_data.circPts.size() >= 3) {
                        // dump circumference points to config file
                        cfg_pts.clear();
                        for (auto p : _input_data.circPts) {
                            cfg_pts.push_back(p.x);
                            cfg_pts.push_back(p.y);
                        }
                        
                        // write to config file
                        BOOST_LOG_TRIVIAL(info) << "Adding roi_circ to config file and writing to disk (" << _config_fn << ")...";
                        _cfg.add("roi_circ", cfg_pts);
                        assert(_cfg.write() > 0);
                        
                        // test read
                        BOOST_LOG_TRIVIAL(debug) << "Re-loading config file and reading roi_circ...";
                        _cfg.read(_config_fn);
                        assert(_cfg.getVecInt("roi_circ", cfg_pts));
                        
                        // advance state
                        cv::destroyWindow("zoomROI");
                        BOOST_LOG_TRIVIAL(debug) << "New state: IGNR_INIT";
                        _input_data.mode = IGNR_INIT;
                    } else {
                        BOOST_LOG_TRIVIAL(warning) << "You must select at least 3 circumference points (you have selected " << _input_data.circPts.size() << " pts)!";
                    }
                }
                break;
            
            /// Check for existing ignore points.
            case IGNR_INIT:
                
                // test read
                cfg_polys.clear();
                if (_cfg.getVVecInt("roi_ignr", cfg_polys)) {
                    
                    /// Load ignore polys from config file.
                    _input_data.ignrPts.clear();
                    for (auto poly : cfg_polys) {
                        _input_data.ignrPts.push_back(vector<cv::Point2d>());
                        for (unsigned int i = 1; i < poly.size(); i+=2) {
                            _input_data.ignrPts.back().push_back(cv::Point2d(poly[i-1],poly[i]));
                        }
                    }
                    
                    /// Draw previous clicks.
                    for (unsigned int i = 0; i < _input_data.ignrPts.size(); i++) {
                        for (unsigned int j = 0; j < _input_data.ignrPts[i].size(); j++) {
                            if (i == _input_data.ignrPts.size()-1) {
                                cv::circle(disp_frame, _input_data.ignrPts[i][j], click_rad, COLOURS[i%NCOLOURS], 1, CV_AA);
                            }
                            cv::line(disp_frame, _input_data.ignrPts[i][j], _input_data.ignrPts[i][(j+1)%_input_data.ignrPts[i].size()], COLOURS[i%NCOLOURS], 1, CV_AA);
                        }
                    }
                    
                    /// Display.
                    cv::imshow("configGUI", disp_frame);
                    cv::waitKey(100);   //FIXME: why do we have to wait so long to make sure the frame is drawn?
                    
                    printf("\n\n\n  Ignore region points were found in the config file. You can discard these points and re-run config or keep the existing points.\n");
                    
                    // input loop
                    while (true) {
                        cv::waitKey(100);   //FIXME: dirty hack - sometimes image doesn't draw, at least with this line we can just mash keys until it does
                        printf("\n  Would you like to keep the existing ignore regions ([y]/n)? ");
                        in = std::getchar();
                        switch (in)
                        {
                            case 'y':
                            case 'Y':
                                std::getchar(); // discard \n
                            case '\n':
                                // advance state
                                BOOST_LOG_TRIVIAL(debug) << "New state: R_INIT";
                                _input_data.mode = R_INIT;
                                break;
                            case 'n':
                            case 'N':
                                std::getchar(); // discard \n
                                break;
                            default:
                                BOOST_LOG_TRIVIAL(error) << "Invalid input!";
                                std::getchar(); // discard \n
                                continue;
                                break;
                        }
                        break;
                    }
                }
                
                if (_input_data.mode == IGNR_INIT) {
                    _input_data.ignrPts.clear();
                    printf("\n\n\n  Define ignore regions.\n\n  Use the left mouse button to add points to a new polygon.\n  Polygons can be drawn around objects (such as the animal) that block the view of the track ball.\n  You can use the right mouse button to remove the last added point.\n\n  Press ENTER to start a new polygon, or press ENTER twice when you are satisfied with the selected ignore regions, or press ESC to exit...\n\n");
                    BOOST_LOG_TRIVIAL(debug) << "New state: IGNR_PTS";
                    _input_data.mode = IGNR_PTS;
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
                
                /// Draw fitted circumference.
                if (r > 0) { drawCircle_camModel(disp_frame, _cam_model, c, r, CV_RGB(255,0,0), false); }
                
                /// Draw cursor location.
                drawCursor(disp_frame, _input_data.cursorPt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS]);
                
                /// Create zoomed window.
                createZoomROI(zoom_frame, disp_frame, _input_data.cursorPt, scaled_zoom_dim);
                
                /// Display.
                cv::imshow("zoomROI", zoom_frame);
                cv::imshow("configGUI", disp_frame);
                key = cv::waitKey(5);
                
                /// State machine logic.
                if (key == enter_key) {
                    // if current poly is empty, assume we've finished
                    if (_input_data.ignrPts.back().empty()) {
                        _input_data.ignrPts.pop_back();
                        
                        // dump ignore region polys to config file
                        cfg_polys.clear();
                        for (auto poly : _input_data.ignrPts) {
                            cfg_polys.push_back(vector<int>());
                            for (auto pt : poly) {
                                cfg_polys.back().push_back(pt.x);
                                cfg_polys.back().push_back(pt.y);
                            }
                        }
                        
                        // write to config file
                        BOOST_LOG_TRIVIAL(info) << "Adding roi_ignr to config file and writing to disk (" << _config_fn << ")...";
                        _cfg.add("roi_ignr", cfg_polys);
                        assert(_cfg.write() > 0);
                        
                        // test read
                        BOOST_LOG_TRIVIAL(debug) << "Re-loading config file and reading roi_ignr...";
                        _cfg.read(_config_fn);
                        assert(_cfg.getVVecInt("roi_ignr", cfg_polys));
                        
                        // advance state
                        cv::destroyWindow("zoomROI");
                        BOOST_LOG_TRIVIAL(debug) << "New state: R_SLCT";
                        _input_data.mode = R_SLCT;
                    }
                    // otherwise, start a new poly
                    else {
                        _input_data.addPoly();
                        BOOST_LOG_TRIVIAL(info) << "New ignore region added!";
                    }
                }
                break;
            
            /// Choose method for defining animal frame.
            case R_INIT:
                // advance state
                BOOST_LOG_TRIVIAL(debug) << "New state: R_SLCT";
                _input_data.mode = R_SLCT;
                break;
            
            /// Choose method for defining animal frame.
            case R_SLCT:
                printf("\n\n\n  Define the animal's coordinate frame.\n\n  You must now define the reference frame of the animal, from the perspective of the camera.\n  This allows FicTrac to convert rotations of the ball into walking and turning motions for the animal.\n");
                printf("  The camera's reference frame is defined as: X = image right (cols); Y = image down (rows); Z = into image (out from camera)\n");
                printf("  The animal's reference frame is defined as: X = forward; Y = right; Z = down\n");
                printf("  Note: the rotational transformation from the camera to the animal reference frame must be accurate, otherwise the data output by FicTrac for the animal will be incorrect!\n");
                
                printf("\n  There are 5 possible methods for defining the animal's coordinate frame:\n");
                printf("\n\t 1 (XY square) : [Default] Click the four corners of a square shape that is aligned with the animal's X-Y axes. This method is recommended when the camera is above/below the animal.\n");
                printf("\n\t 2 (YZ square) : Click the four corners of a square shape that is aligned with the animal's Y-Z axes. This method is recommended when the camera is in front/behind the animal.\n");
                printf("\n\t 3 (XZ square) : Click the four corners of a square shape that is aligned with the animal's X-Y axes. This method is recommended when the camera is to the animal's left/right.\n");
                printf("\n\t 4 (manual)    : Rotate a visualisation of the animal's coordinate frame to align with the orientation of the animal. This method is not recommended as it is inaccurate.\n");
                printf("\n\t 5 (external)  : The transform between the camera and animal reference frames can also be defined by hand by editing the appropriate variables in the config file. This method is only recommended when the transform is known by some other means.\n");
                
                // input loop
                while (true) {
                    printf("\n\n  Please enter your preferred method [1]: ");
                    std::getline(std::cin, str);
                    if (str.empty()) {
                        in = 1;
                    } else {
                        try { in = std::stoi(str); }
                        catch(...) {
                            BOOST_LOG_TRIVIAL(error) << "Invalid input!";
                            continue;
                        }
                    }
                    switch (in)
                    {
                        case 1:
                            printf("\n\n\n  XY Square method.\n\n  Please click on the four corners of a square shape that is aligned with the animal's X-Y axes. The corners must be clicked in the following order: (+X,-Y), (+X,+Y), (-X,+Y), (-X,-Y). If your camera is looking down on the animal from above, then the four corners are (in order): TL, TR, BR, BL from the camera's perspective. If your camera is below the animal, then the order is TR, TL, BL, BR.\n\n  Press ENTER when you are satisfied with the selected corners, or press ESC to exit...\n\n");
                            // advance state
                            BOOST_LOG_TRIVIAL(debug) << "New state: R_XY";
                            _input_data.mode = R_XY;
                            break;
                            
                        case 2:
                            printf("\n\n\n  YZ Square method.\n\n  Please click on the four corners of a square shape that is aligned with the animal's Y-Z axes. The corners must be clicked in the following order: (-Y,-Z), (+Y,-Z), (+Y,+Z), (-Y,+Z). If your camera is behind the animal, then the four corners are (in order): TL, TR, BR, BL from the camera's perspective. If your camera is in front of the animal, then the order is TR, TL, BL, BR.\n\n  Press ENTER when you are satisfied with the selected corners, or press ESC to exit...\n\n");
                            // advance state
                            BOOST_LOG_TRIVIAL(debug) << "New state: R_YZ";
                            _input_data.mode = R_YZ;
                            break;
                            
                        case 3:
                            printf("\n\n\n  XZ Square method.\n\n  Please click on the four corners of a square shape that is aligned with the animal's X-Y axes. The corners must be clicked in the following order: (+X,-Z), (-X,-Z), (-X,+Z), (+X,+Z). If your camera is to the animal's left side, then the four corners are (in order): TL, TR, BR, BL from the camera's perspective. If your camera is to the animal's right side, then the order is TR, TL, BL, BR.\n\n  Press ENTER when you are satisfied with the selected corners, or press ESC to exit...\n\n");
                            // advance state
                            BOOST_LOG_TRIVIAL(debug) << "New state: R_XZ";
                            _input_data.mode = R_XZ;
                            break;
                            
                        case 4:
                            // advance state
                            BOOST_LOG_TRIVIAL(debug) << "New state: R_MAN";
                            _input_data.mode = R_MAN;
                            break;
                            
                        case 5:
                            // advance state
                            BOOST_LOG_TRIVIAL(debug) << "New state: R_EXT";
                            _input_data.mode = R_EXT;
                            break;
                            
                        default:
                            BOOST_LOG_TRIVIAL(error) << "Invalid input!";
                            continue;
                            break;
                    }
                    break;
                }
                break;
            
            /// Define animal coordinate frame.
            case R_XY:
            
                /// Draw previous clicks.
                for (auto click : _input_data.sqrePts) {
                    cv::circle(disp_frame, click, click_rad, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS], 1, CV_AA);
                }
                
                /// Draw cursor location.
                drawCursor(disp_frame, _input_data.cursorPt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS]);
                
                /// Create zoomed window.
                createZoomROI(zoom_frame, disp_frame, _input_data.cursorPt, scaled_zoom_dim);
                
                /// Display.
                cv::imshow("zoomROI", zoom_frame);
                cv::imshow("configGUI", disp_frame);
                key = cv::waitKey(5);
                
                /// State machine logic.
                if (key == enter_key) {
                    if (_input_data.sqrePts.size() == 4) {
                        // dump corner points to config file
                        cfg_pts.clear();
                        for (auto p : _input_data.sqrePts) {
                            cfg_pts.push_back(p.x);
                            cfg_pts.push_back(p.y);
                        }
                        
                        // write to config file
                        BOOST_LOG_TRIVIAL(info) << "Adding sqr_cnrs_xy to config file and writing to disk (" << _config_fn << ")...";
                        _cfg.add("sqr_cnrs_xy", cfg_pts);
                        assert(_cfg.write() > 0);
                        
                        // test read
                        BOOST_LOG_TRIVIAL(debug) << "Re-loading config file and reading roi_circ...";
                        _cfg.read(_config_fn);
                        assert(_cfg.getVecInt("sqr_cnrs_xy", cfg_pts));
                        
                        // advance state
                        cv::destroyWindow("zoomROI");
                        BOOST_LOG_TRIVIAL(debug) << "New state: EXIT";
                        _input_data.mode = EXIT;
                    } else {
                        BOOST_LOG_TRIVIAL(warning) << "You must select exactly 4 corners (you have selected " << _input_data.sqrePts.size() << " pts)!";
                    }
                }
                break;
            
            /// Define animal coordinate frame.
            case R_YZ:
                
                /// Draw previous clicks.
                for (auto click : _input_data.sqrePts) {
                    cv::circle(disp_frame, click, click_rad, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS], 1, CV_AA);
                }
                
                /// Draw axes.
                if (_input_data.sqrePts.size() == 4) {
                    Mat R;
                    if (computeRFromSquare_YZ(_cam_model, _input_data.sqrePts, R)) {
                        
                        Mat o = (cv::Mat_<float>(3,1) << c.x, c.y, c.z);
                        Mat x = o + R * (cv::Mat_<float>(3,1) << 1,0,0) * 0.25;
                        Mat y = o + R * (cv::Mat_<float>(3,1) << 0,1,0) * 0.25;
                        Mat z = o + R * (cv::Mat_<float>(3,1) << 0,0,1) * 0.25;
                        
                        vec[0] = o.at<float>(0,0);
                        vec[1] = o.at<float>(1,0);
                        vec[2] = o.at<float>(2,0);
                        _cam_model->vectorToPixel(vec, pt0.x, pt0.y);
                        
                        vec[0] = x.at<float>(0,0);
                        vec[1] = x.at<float>(1,0);
                        vec[2] = x.at<float>(2,0);
                        _cam_model->vectorToPixel(vec, pt.x, pt.y);
                        
                        cv::line(disp_frame, 4*pt0, 4*pt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS], 1, CV_AA, 2);
                        
                        vec[0] = y.at<float>(0,0);
                        vec[1] = y.at<float>(1,0);
                        vec[2] = y.at<float>(2,0);
                        _cam_model->vectorToPixel(vec, pt.x, pt.y);
                        
                        cv::line(disp_frame, 4*pt0, 4*pt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS], 1, CV_AA, 2);
                        
                        vec[0] = z.at<float>(0,0);
                        vec[1] = z.at<float>(1,0);
                        vec[2] = z.at<float>(2,0);
                        _cam_model->vectorToPixel(vec, pt.x, pt.y);
                        
                        cv::line(disp_frame, 4*pt0, 4*pt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS], 1, CV_AA, 2);
                    }
                }
                
                /// Draw cursor location.
                drawCursor(disp_frame, _input_data.cursorPt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS]);
                
                /// Create zoomed window.
                createZoomROI(zoom_frame, disp_frame, _input_data.cursorPt, scaled_zoom_dim);
                
                /// Display.
                cv::imshow("zoomROI", zoom_frame);
                cv::imshow("configGUI", disp_frame);
                key = cv::waitKey(5);
                
                /// State machine logic.
                if (key == enter_key) {
                    if (_input_data.sqrePts.size() == 4) {
                        // dump corner points to config file
                        cfg_pts.clear();
                        for (auto p : _input_data.sqrePts) {
                            cfg_pts.push_back(p.x);
                            cfg_pts.push_back(p.y);
                        }
                        
                        // write to config file
                        BOOST_LOG_TRIVIAL(info) << "Adding sqr_cnrs_yz to config file and writing to disk (" << _config_fn << ")...";
                        _cfg.add("sqr_cnrs_yz", cfg_pts);
                        assert(_cfg.write() > 0);
                        
                        // test read
                        BOOST_LOG_TRIVIAL(debug) << "Re-loading config file and reading roi_circ...";
                        _cfg.read(_config_fn);
                        assert(_cfg.getVecInt("sqr_cnrs_yz", cfg_pts));
                        
                        // advance state
                        cv::destroyWindow("zoomROI");
                        BOOST_LOG_TRIVIAL(debug) << "New state: EXIT";
                        _input_data.mode = EXIT;
                    } else {
                        BOOST_LOG_TRIVIAL(warning) << "You must select exactly 4 corners (you have selected " << _input_data.sqrePts.size() << " pts)!";
                    }
                }
                break;
            
            /// Define animal coordinate frame.
            case R_XZ:
                
                /// Draw previous clicks.
                for (auto click : _input_data.sqrePts) {
                    cv::circle(disp_frame, click, click_rad, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS], 1, CV_AA);
                }
                
                /// Draw cursor location.
                drawCursor(disp_frame, _input_data.cursorPt, COLOURS[static_cast<int>(_input_data.mode)%NCOLOURS]);
                
                /// Create zoomed window.
                createZoomROI(zoom_frame, disp_frame, _input_data.cursorPt, scaled_zoom_dim);
                
                /// Display.
                cv::imshow("zoomROI", zoom_frame);
                cv::imshow("configGUI", disp_frame);
                key = cv::waitKey(5);
                
                /// State machine logic.
                if (key == enter_key) {
                    if (_input_data.sqrePts.size() == 4) {
                        // dump corner points to config file
                        cfg_pts.clear();
                        for (auto p : _input_data.sqrePts) {
                            cfg_pts.push_back(p.x);
                            cfg_pts.push_back(p.y);
                        }
                        
                        // write to config file
                        BOOST_LOG_TRIVIAL(info) << "Adding sqr_cnrs_xz to config file and writing to disk (" << _config_fn << ")...";
                        _cfg.add("sqr_cnrs_xz", cfg_pts);
                        assert(_cfg.write() > 0);
                        
                        // test read
                        BOOST_LOG_TRIVIAL(debug) << "Re-loading config file and reading roi_circ...";
                        _cfg.read(_config_fn);
                        assert(_cfg.getVecInt("sqr_cnrs_xz", cfg_pts));
                        
                        // advance state
                        cv::destroyWindow("zoomROI");
                        BOOST_LOG_TRIVIAL(debug) << "New state: EXIT";
                        _input_data.mode = EXIT;
                    } else {
                        BOOST_LOG_TRIVIAL(warning) << "You must select exactly 4 corners (you have selected " << _input_data.sqrePts.size() << " pts)!";
                    }
                }
                break;
            
            /// Define animal coordinate frame.
            case R_MAN:
                
                // advance state
                BOOST_LOG_TRIVIAL(debug) << "New state: EXIT";
                _input_data.mode = EXIT;
                break;
            
            /// Define animal coordinate frame.
            case R_EXT:
            
                // advance state
                BOOST_LOG_TRIVIAL(debug) << "New state: EXIT";
                _input_data.mode = EXIT;
                break;
            
            /// Exit config.
            case EXIT:
                key = exit_key;
                break;
            
            default:
                BOOST_LOG_TRIVIAL(debug) << "Unexpected state encountered!";
                _input_data.mode = EXIT;
                break;
        }
        if (key == exit_key) { break; }
        
        
    }
    
    BOOST_LOG_TRIVIAL(info) << "Exiting configGui!";
    
    cv::destroyAllWindows();
    return _open;
}