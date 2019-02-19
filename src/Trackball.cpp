/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Trackball.cpp
/// \brief      Stores surface map and current orientation of tracking ball.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//FIXME: better alg for preferancing high-overlap good matches?

#include "Trackball.h"

#include "geometry.h"
#include "drawing.h"
#include "Logger.h"
#include "timing.h"
#include "CameraRemap.h"
#include "BasicRemapper.h"
#include "misc.h"
#include "CVSource.h"
#ifdef PGR_USB3
#include "PGRSource.h"
#endif // PGR_USB3

/// OpenCV individual includes required by gcc?
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <sstream>
#include <cmath>
#include <exception>

using std::string;
using std::make_unique;
using std::make_shared;
using std::unique_lock;
using std::lock_guard;
using std::mutex;
using std::vector;
using std::deque;
using cv::Mat;
using cv::Scalar;
using cv::Rect;
using cv::Point2d;
using cv::Point2i;

const int DRAW_SPHERE_HIST_LENGTH = 250;
const int DRAW_CELL_DIM = 160;
const int DRAW_FICTIVE_PATH_LENGTH = 1000;

const int Q_FACTOR_DEFAULT = 6;
const double OPT_TOL_DEFAULT = 1e-3;
const double OPT_BOUND_DEFAULT = 0.35;
const int OPT_MAX_EVAL_DEFAULT = 50;
const bool OPT_GLOBAL_SEARCH_DEFAULT = false;
const int OPT_MAX_BAD_FRAMES_DEFAULT = -1;

const double THRESH_RATIO_DEFAULT = 1.25;
const double THRESH_WIN_PC_DEFAULT = 0.25;

const uint8_t SPHERE_MAP_FIRST_HIT_BONUS = 64;

const int OUT_PORT_DEFAULT = -1;

const bool DO_DISPLAY_DEFAULT = true;
const bool SAVE_RAW_DEFAULT = false;
const bool SAVE_DEBUG_DEFAULT = false;

///
///
///
bool intersectSphere(const double camVec[3], double sphereVec[3], const double r)
{
    double q = camVec[2] * camVec[2] + r * r - 1;
    if (q < 0) { return false; }

    // get point on front surface of sphere (camera coords)
    double u = camVec[2] - sqrt(q);

    // switch to sphere coords
    sphereVec[0] = camVec[0] * u;
    sphereVec[1] = camVec[1] * u;
    sphereVec[2] = camVec[2] * u - 1;

    return true;
}

///
/// 
///
Trackball::Trackball(string cfg_fn)
    : _init(false), _reset(true), _clean_map(true), _active(true), _kill(false)
{
    /// Load and parse config file.
    if (_cfg.read(cfg_fn) <= 0) {
        LOG_ERR("Error parsing config file (%s)!", cfg_fn.c_str());
        _active = false;
        return;
    }

    /// Open frame source and set fps.
    string src_fn = _cfg("src_fn");
    std::shared_ptr<FrameSource> source;
#ifdef PGR_USB3
    try {
        // first try reading input as camera id
        int id = std::stoi(src_fn);
        source = make_shared<PGRSource>(id);
    }
    catch (...) {
        // then try loading as video file
        source = make_shared<CVSource>(src_fn);
    }
#else // !PGR_USB3
    source = std::make_shared<CVSource>(src_fn);
#endif // PGR_USB3
    if (!source->isOpen()) {
        LOG_ERR("Error! Could not open input frame source (%s)!", src_fn.c_str());
        _active = false;
        return;
    }
    double src_fps = -1;
    if (_cfg.getDbl("src_fps", src_fps) && (src_fps > 0)) {
        LOG("Setting source fps = %.2f..", src_fps);
        source->setFPS(src_fps);
    }
    else {
        _cfg.add("src_fps", src_fps);
    }

    /// Create base file name for output files.
    _base_fn = _cfg("output_fn");
    if (_base_fn.empty()) {
        if (!source->isLive()) {
            _base_fn = src_fn.substr(0, src_fn.length() - 4);
        } else {
            _base_fn = "fictrac";
        }
    }

    /// Source camera model.
    double vfov = -1;
    if (!_cfg.getDbl("vfov", vfov) || (vfov <= 0)) {
        LOG_ERR("Error! Camera vertical FoV parameter specified in the config file (vfov) is invalid!");
        _active = false;
        return;
    }
    //FIXME: support other camera models
    _src_model = CameraModel::createRectilinear(source->getWidth(), source->getHeight(), vfov * CM_D2R);

    /// Dimensions - quality defaults to 6 (remap_dim 60x60, sphere_dim 180x90).
    int q_factor = Q_FACTOR_DEFAULT;
    if (!_cfg.getInt("q_factor", q_factor) || (q_factor <= 0)) {
        LOG_WRN("Warning! Resolution parameter specified in the config file (q_factor) is invalid! Using default value (%d).", q_factor);
        _cfg.add("q_factor", q_factor);
    }
    _roi_w = _roi_h = std::min(10 * q_factor,source->getWidth());
    _map_h = static_cast<int>(1.5 * _roi_h);
    _map_w = 2 * _map_h;

    /// Load sphere config and mask.
    Mat src_mask(source->getHeight(), source->getWidth(), CV_8UC1);
    src_mask.setTo(Scalar::all(0));
    {
        // read pts from config file
        _sphere_rad = -1;
        vector<int> circ_pxs;
        vector<double> sphere_c;
        if (_cfg.getVecDbl("roi_c", sphere_c) && _cfg.getDbl("roi_r", _sphere_rad)) {
            _sphere_c.copy(sphere_c.data());
            LOG("Found sphere ROI centred at [%f %f %f], with radius %f rad.", _sphere_c[0], _sphere_c[1], _sphere_c[2], _sphere_rad);
        }
        else if (_cfg.getVecInt("roi_circ", circ_pxs)) {
            vector<Point2d> circ_pts;
            for (unsigned int i = 1; i < circ_pxs.size(); i += 2) {
                circ_pts.push_back(Point2d(circ_pxs[i - 1], circ_pxs[i]));
            }

            // fit circular fov
            if ((circ_pts.size() >= 3) && circleFit_camModel(circ_pts, _src_model, _sphere_c, _sphere_rad)) {
                LOG("Computed sphere ROI centred at [%f %f %f], with radius %f rad from %d roi_circ points.",
                    _sphere_c[0], _sphere_c[1], _sphere_c[2], _sphere_rad, circ_pts.size());
            }
        }

        if (_sphere_rad > 0) {
            // NOTE: sin rather than tan to correct for apparent size of sphere
            _r_d_ratio = sin(_sphere_rad);

            /// Allow sphere region in mask.
            auto int_circ = projCircleInt(_src_model, _sphere_c, _sphere_rad);
            cv::fillConvexPoly(src_mask, *int_circ, CV_RGB(255, 255, 255));

            /// Mask out ignore regions.
            vector<vector<int>> ignr_polys;
            if (_cfg.getVVecInt("roi_ignr", ignr_polys) && (ignr_polys.size() > 0)) {
                /// Load ignore polys from config file.
                vector<vector<Point2i>> ignr_polys_pts;
                for (auto poly : ignr_polys) {
                    ignr_polys_pts.push_back(vector<Point2i>());
                    for (unsigned int i = 1; i < poly.size(); i += 2) {
                        ignr_polys_pts.back().push_back(Point2i(poly[i - 1], poly[i]));
                    }
                }

                /// Fill ignore region polys.
                cv::fillPoly(src_mask, ignr_polys_pts, CV_RGB(0, 0, 0));
            }
            else {
                LOG_WRN("Warning! No valid mask ignore regions specified in config file (roi_ignr)!");
            }
                
            /// Sphere config read successfully.
            LOG("Input sphere mask automatically generated using %d ignore ROIs!", ignr_polys.size());
        }
        else {
            LOG_ERR("Error! Sphere ROI configuration specified in config file (roi_circ, roi_c, roi_r) is invalid!");
            _active = false;
            return;
        }
    }

    /*
    * The model sphere has it's own coordinate system, because we arbitrarily set
    * the view vector corresponding to the centre of the projection of the tracking
    * ball to be the principal axis of the virtual camera (cam_model).
    *
    * All incoming and outgoing vectors and matrices must thus be transposed to/from
    * this coordinate system: lab <= (_cam_to_lab_R) => cam <= (_roi_to_cam_R) => roi.
    */

    /// Create coordinate frame transformation matrices.
    CmPoint64f roi_to_cam_r, cam_to_lab_r;
    {
        // ROI to cam transformation from sphere centre ray.
        CmPoint64f z(0, 0, 1);      // forward in camera coords
        roi_to_cam_r = _sphere_c.getRotationTo(z);    // find axis-angle to rotate sphere centre to camera centre.
        _roi_to_cam_R = CmPoint64f::omegaToMatrix(roi_to_cam_r);

        LOG_DBG("roi_to_cam_r: %.4f %.4f %.4f", roi_to_cam_r[0], roi_to_cam_r[1], roi_to_cam_r[2]);

        // Cam to lab transformation from configuration.
        vector<double> c2a_r;
        if (_cfg.getVecDbl("c2a_r", c2a_r) && (c2a_r.size() == 3)) {
            cam_to_lab_r = CmPoint64f(c2a_r[0], c2a_r[1], c2a_r[2]);
            _cam_to_lab_R = CmPoint64f::omegaToMatrix(cam_to_lab_r);
        }
        else {
            LOG_ERR("Error! Camera-to-lab coordinate tranformation specified in config file (c2a_r) is invalid!");
            _active = false;
            return;
        }
    }

    ///// Remap (ROI) model and remapper.
    double sphere_radPerPix = _sphere_rad * 2.0 / _roi_w;
    _roi_model = CameraModel::createFisheye(_roi_w, _roi_h, sphere_radPerPix, _sphere_rad * 2.0);
    _cam_to_roi = MatrixRemapTransform::createFromOmega(-roi_to_cam_r);
    CameraRemapPtr remapper = CameraRemapPtr(new CameraRemap(_src_model, _roi_model, _cam_to_roi));

    /// ROI mask.
    _roi_mask.create(_roi_h, _roi_w, CV_8UC1);
    _roi_mask.setTo(cv::Scalar::all(255));
    remapper->apply(src_mask, _roi_mask);
    erode(_roi_mask, _roi_mask, Mat(), cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);   // remove edge effects

    /// Surface mapping.
    _sphere_model = CameraModel::createEquiArea(_map_w, _map_h);

    /// Buffers.
    _sphere_map.create(_map_h, _map_w, CV_8UC1);
    _sphere_map.setTo(cv::Scalar::all(128));

    /// Surface map template.
    _sphere_template = _sphere_map.clone();
    {
        string sphere_template_fn;
        if (_cfg.getStr("sphere_map_fn", sphere_template_fn)) {
            _sphere_template = cv::imread(sphere_template_fn, 0);
            if ((_sphere_template.cols != _map_w) || (_sphere_template.rows != _map_h)) {
                LOG_ERR("Error! Sphere map template specified in the config file (sphere_map_fn) is invalid (%dx%d)!", _sphere_template.cols, _sphere_template.rows);
                _active = false;
                return;
            }

            /// Store initial sphere map.
            _sphere_template.copyTo(_sphere_map);
            _clean_map = false;

            LOG("Loaded initial sphere template from %s.", sphere_template_fn.c_str());
        }
    }

    /// Pre-calc view rays.
    _p1s_lut = std::shared_ptr<double[]>(new double[_roi_w * _roi_h * 3]);
    memset(_p1s_lut.get(), 0, _roi_w * _roi_h * 3 * sizeof(double));
    for (int i = 0; i < _roi_h; i++) {
        uint8_t* pmask = _roi_mask.ptr(i);
        for (int j = 0; j < _roi_w; j++) {
            if (pmask[j] < 255) { continue; }

            double l[3] = { 0 };
            _roi_model->pixelIndexToVector(j, i, l);
            vec3normalise(l);

            double* s = &_p1s_lut[(i * _roi_w + j) * 3];
            if (!intersectSphere(l, s, _r_d_ratio)) { pmask[j] = 128; }
        }
    }

    /// Read config params.
    double tol = OPT_TOL_DEFAULT;
    if (!_cfg.getDbl("opt_tol", tol) || (tol <= 0)) {
        LOG_WRN("Warning! Using default value for opt_tol (%f).", tol);
        _cfg.add("opt_tol", tol);
    }
    double bound = OPT_BOUND_DEFAULT;
    if (!_cfg.getDbl("opt_bound", bound) || (bound <= 0)) {
        LOG_WRN("Warning! Using default value for opt_bound (%f).", bound);
        _cfg.add("opt_bound", bound);
    }
    int max_evals = OPT_MAX_EVAL_DEFAULT;
    if (!_cfg.getInt("opt_max_evals", max_evals) || (max_evals <= 0)) {
        LOG_WRN("Warning! Using default value for opt_max_eval (%d).", max_evals);
        _cfg.add("opt_max_evals", max_evals);
    }
    _global_search = OPT_GLOBAL_SEARCH_DEFAULT;
    if (!_cfg.getBool("opt_do_global", _global_search)) {
        LOG_WRN("Warning! Using default value for opt_do_global (%d).", _global_search);
        _cfg.add("opt_do_global", _global_search);
    }
    _max_bad_frames = OPT_MAX_BAD_FRAMES_DEFAULT;
    if (!_cfg.getInt("max_bad_frames", _max_bad_frames)) {
        LOG_WRN("Warning! Using default value for max_bad_frames (%d).", _max_bad_frames);
        _cfg.add("max_bad_frames", _max_bad_frames);
    }
    _error_thresh = -1;
    if (!_cfg.getDbl("opt_max_err", _error_thresh) || (_error_thresh < 0)) {
        LOG_WRN("Warning! No optimisation error threshold specified in config file (opt_max_err) - poor matches will not be dropped!");
        _cfg.add("opt_max_err", _error_thresh);
    }
    double thresh_ratio = THRESH_RATIO_DEFAULT;
    if (!_cfg.getDbl("thr_ratio", thresh_ratio)) {
        LOG_WRN("Warning! Using default value for thr_ratio (%f).", thresh_ratio);
        _cfg.add("thr_ratio", thresh_ratio);
    }
    double thresh_win_pc = THRESH_WIN_PC_DEFAULT;
    if (!_cfg.getDbl("thr_win_pc", thresh_win_pc)) {
        LOG_WRN("Warning! Using default value for thr_win_pc (%f).", thresh_win_pc);
        _cfg.add("thr_win_pc", thresh_win_pc);
    }

    /// Init optimisers.
    _localOpt = make_unique<Localiser>(
        NLOPT_LN_BOBYQA, bound, tol, max_evals,
        _sphere_model, _sphere_map,
        _roi_mask, _p1s_lut);

    _globalOpt = make_unique<Localiser>(
        NLOPT_GN_CRS2_LM, CM_PI, tol, 1e5,
        _sphere_model, _sphere_map,
        _roi_mask, _p1s_lut);

    /// Output.
    string data_fn = _base_fn + "-" + execTime() + ".dat";
    _data_log = make_unique<Recorder>(RecorderInterface::RecordType::FILE, data_fn);
    if (!_data_log->is_active()) {
        LOG_ERR("Error! Unable to open output data log file (%s).", data_fn.c_str());
        _active = false;
        return;
    }

    int out_port = OUT_PORT_DEFAULT;
    _do_sock_output = false;
    if (_cfg.getInt("out_port", out_port) && (out_port > 0)) {
        _data_sock = make_unique<Recorder>(RecorderInterface::RecordType::SOCK, std::to_string(out_port));
        if (!_data_sock->is_active()) {
            LOG_ERR("Error! Unable to open output data socket (%d).", out_port);
            _active = false;
            return;
        }
        _do_sock_output = true;
    } else {
        LOG_WRN("Warning! Using default value for out_port (%d).", out_port);
        _cfg.add("out_port", out_port);
    }

    /// Display.
    _do_display = DO_DISPLAY_DEFAULT;
    if (!_cfg.getBool("do_display", _do_display)) {
        LOG_WRN("Warning! Using default value for do_display (%d).", _do_display);
        _cfg.add("do_display", _do_display);
    }
    _save_raw = SAVE_RAW_DEFAULT;
    if (!source->isLive() || !_cfg.getBool("save_raw", _save_raw)) {
        LOG_WRN("Warning! Using default value for save_raw (%d).", _save_raw);
        _cfg.add("save_raw", _save_raw);
    }
    _save_debug = SAVE_DEBUG_DEFAULT;
    if (!_cfg.getBool("save_debug", _save_debug)) {
        LOG_WRN("Warning! Using default value for save_debug (%d).", _save_debug);
        _cfg.add("save_debug", _save_debug);
    }
    if (_save_debug & !_do_display) {
        LOG("Forcing do_display = true, becase save_debug == true.");
        _do_display = true;
    }
    if (_do_display) {
        _sphere_view.create(_map_h, _map_w, CV_8UC1);
        _sphere_view.setTo(Scalar::all(128));
    }
    if (_save_raw) {
        string vid_fn = _base_fn + "-raw.mp4";
        double fps = source->getFPS();
        if (fps <= 0) { fps = 25; }
        LOG_DBG("Opening %s for video writing (%dx%d @ %f FPS)", vid_fn.c_str(), source->getWidth(), source->getHeight(), fps);
        _raw_vid.open(vid_fn, cv::VideoWriter::fourcc('H', '2', '6', '4'), fps, cv::Size(source->getWidth(), source->getHeight()));
        if (!_raw_vid.isOpened()) {
            LOG_ERR("Error! Unable to open raw output video (%s).", vid_fn.c_str());
            _active = false;
            return;
        }
    }
    if (_save_debug) {
        string vid_fn = _base_fn + "-debug.mp4";
        double fps = source->getFPS();
        if (fps <= 0) { fps = 25; }
        LOG_DBG("Opening %s for video writing (%dx%d @ %f FPS)", vid_fn.c_str(), 4 * DRAW_CELL_DIM, 3 * DRAW_CELL_DIM, fps);
        _debug_vid.open(vid_fn, cv::VideoWriter::fourcc('H', '2', '6', '4'), fps, cv::Size(4 * DRAW_CELL_DIM, 3 * DRAW_CELL_DIM));
        if (!_debug_vid.isOpened()) {
            LOG_ERR("Error! Unable to open debug output video (%s).", vid_fn.c_str());
            _active = false;
            return;
        }
    }

    /// Frame source.
    _frameGrabber = std::make_unique<FrameGrabber>(
        source,
        remapper,
        _roi_mask,
        thresh_ratio,
        thresh_win_pc
    );

    /// Write all parameters back to config file.
    _cfg.write();

    /// Data.
    _cnt = 0;
    _err = 0;
    _intx = _inty = 0;
    reset();

    /// Thread stuff.
    _init = true;
    _active = true;

    if (_do_display) {
        _drawThread = make_unique<std::thread>(&Trackball::processDrawQ, this);
    }
    // main processing thread
    _thread = make_unique<std::thread>(&Trackball::process, this);
}

///
/// Default destructor.
///
Trackball::~Trackball()
{
    LOG("Closing sphere tracker..");

    _init = false;
    _active = false;

    if (_thread && _thread->joinable()) {
        _thread->join();
    }

    if (_do_display && _drawThread && _drawThread->joinable()) {
        _drawThread->join();
    }
}

///
///
///
void Trackball::reset()
{
    _reset = true;

    /// Clear maps if we can't search the entire sphere to relocalise.
    if (!_global_search) {
        //FIXME: possible for users to specify sphere_template without enabling global search..
        _sphere_template.copyTo(_sphere_map);
        _clean_map = true;
    }

    /// Reset sphere.
    _R_roi = Mat::eye(3, 3, CV_64F);

    /// Reset data.
    _seq = 0;       // indicates new sequence started
    _posx = 0;      // reset because heading was lost
    _posy = 0;
    _heading = 0;

    // test data
    _dist = 0;
    _ang_dist = 0;
    _step_avg = 0;
    _step_var = 0;
    _evals_avg = 0;

    /// Drawing.
    if (_do_display) {
        _R_roi_hist.clear();
        _pos_heading_hist.clear();
    }
}

///
///
///
void Trackball::process()
{
    LOG_DBG("Starting sphere tracking loop!");

    /// Set thread high priority (when run as SU).
    if (!SetThreadHighPriority()) {
        LOG_ERR("Error! Unable to set thread priority!");
    } else {
        LOG("Set processing thread priority to HIGH!");
    }

    /// Sphere tracking loop.
    int nbad = 0;
    double t0 = ts_ms();
    double t1, t2, t3, t4, t5, t6;
    double t1avg = 0, t2avg = 0, t3avg = 0, t4avg = 0, t5avg = 0, t6avg = 0;
    double tfirst = -1, tlast = 0;
    while (!_kill && _active && _frameGrabber->getNextFrameSet(_src_frame, _roi_frame, _ts)) {
        t1 = ts_ms();

        PRINT("");
        LOG("Frame %d", _cnt);

        /// Localise current view of sphere.
        if (!doSearch(_global_search)) {
            t2 = t3 = t4 = t5 = ts_ms();
            LOG_WRN("Warning! Could not match current sphere orientation to within error threshold (%f).\nNo data will be output for this frame!", _error_thresh);
            nbad++;
        }
        else {
            /// Clear reset flag.
            _reset = false;

            t2 = ts_ms();
            updateSphere();
            t3 = ts_ms();
            updatePath();
            t4 = ts_ms();
            logData();  // only output good data
            t5 = ts_ms();
            nbad = 0;
        }

        /// Handle failed localisation.
        if ((_max_bad_frames >= 0) && (nbad > _max_bad_frames)) {
            _seq = 0;
            nbad = 0;
            reset();
        } else {
            _seq++;
        }

        if (_do_display) {
            auto data = make_shared<DrawData>();
            data->src_frame = _src_frame.clone();
            data->roi_frame = _roi_frame.clone();
            data->sphere_map = _sphere_map.clone();
            data->sphere_view = _sphere_view.clone();
            data->dr_roi = _dr_roi;
            data->R_roi = _R_roi.clone();
            data->R_roi_hist = _R_roi_hist;
            data->pos_heading_hist = _pos_heading_hist;

            updateCanvasAsync(data);
        }
        t6 = ts_ms();

        /// Timing.
        if (_cnt > 0) {     // skip first frame (often global search...)
            t1avg += t1 - t0;
            t2avg += t2 - t1;
            t3avg += t3 - t2;
            t4avg += t4 - t3;
            t5avg += t5 - t4;
            t6avg += t6 - t5;

            // opt evals
            _evals_avg += _nevals;
        }
        LOG("Timing grab/opt/map/plot/log/disp: %.1f / %.1f / %.1f / %.1f / %.1f / %.1f ms",
            t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4, t6 - t5);
        static double prev_t6 = t6;
        double fps_out = (t6 - prev_t6) > 0 ? 1000 / (t6 - prev_t6) : 0;
        static double fps_avg = fps_out;
        fps_avg += 0.25 * (fps_out - fps_avg);
        static double prev_ts = _ts;
        double fps_in = (_ts - prev_ts) > 0 ? 1000 / (_ts - prev_ts) : 0;
        LOG("Average frame rate [in/out]: %.1f [%.1f / %.1f] fps", fps_avg, fps_in, fps_out);
        prev_t6 = t6;
        prev_ts = _ts;

        /// Always increment frame counter.
        _cnt++;

        t0 = ts_ms();
        if (tfirst < 0) { tfirst = t0; }
    }
    tlast = t0;

    LOG_DBG("Stopped sphere tracking loop!");

    _frameGrabber->terminate();     // make sure we've stopped grabbing frames as well

    if (_cnt > 1) {
        PRINT("\n----------------------------------------------------------------------------");
        LOG("Trackball timing:");
        LOG("Average grab/opt/map/plot/log/disp time: %.1f / %.1f / %.1f / %.1f / %.1f / %.1f ms",
            t1avg / (_cnt - 1), t2avg / (_cnt - 1), t3avg / (_cnt - 1), t4avg / (_cnt - 1), t5avg / (_cnt - 1), t6avg / (_cnt - 1));
        LOG("Average fps: %.2f", 1000. * (_cnt - 1) / (tlast - tfirst));

        PRINT("");
        LOG("Optimiser test data:");
        LOG("Average number evals / frame: %.1f", _evals_avg / (_cnt - 1));
        PRINT("----------------------------------------------------------------------------");
    }

    _active = false;
}

///
///
///
bool Trackball::doSearch(bool allow_global = false)
{
    /// Maintain a low-pass filtered rotation to use as guess.
    static CmPoint64f guess(0, 0, 0);
    if (_reset) { guess = CmPoint64f(0, 0, 0); }

    /// Run optimisation and save result.
    _nevals = 0;
    if (!_reset) {
        _dr_roi = guess;
        _err = _localOpt->search(_roi_frame, _R_roi, _dr_roi);  // _dr_roi contains optimal rotation
        _nevals = _localOpt->getNumEval();
    }
    else {
        _dr_roi = CmPoint64f(0, 0, 0);
        _err = 0;
    }

    /// Check optimisation.
    bool bad_frame = _error_thresh >= 0 ? (_err > _error_thresh) : false;
    if (allow_global && (bad_frame || (_reset && !_clean_map))) {

        LOG("Doing global search..");

        // do global search
        _err = _globalOpt->search(_roi_frame, _R_roi, _r_roi); // use last know orientation, _r_roi, as guess and update with result
        _nevals = _globalOpt->getNumEval();
        bad_frame = _error_thresh >= 0 ? (_err > _error_thresh) : false;

        // if global search failed as well, just reset global orientation too
        if (bad_frame) {
            _r_roi = CmPoint64f(0, 0, 0);  // zero absolute orientation
        }
        
        // reset sphere to found orientation with zero motion
        _dr_roi = CmPoint64f(0, 0, 0);  // zero relative rotation
        _R_roi = CmPoint64f::omegaToMatrix(_r_roi);
    }
    else {
        /// Accumulate sphere orientation.
        Mat tmpR = CmPoint64f::omegaToMatrix(_dr_roi);      // relative rotation (angle-axis) in ROI frame
        _R_roi = tmpR * _R_roi;                             // pre-multiply to accumulate orientation matrix
        _r_roi = CmPoint64f::matrixToOmega(_R_roi);
    }

    LOG("optimum sphere rotation:\t%.3f %.3f %.3f  (err=%.3e/its=%d)", _dr_roi[0], _dr_roi[1], _dr_roi[2], _err, _nevals);
    LOG_DBG("Current sphere orientation:\t%.3f %.3f %.3f", _r_roi[0], _r_roi[1], _r_roi[2]);

    if (!bad_frame) {
        guess = 0.9 * _dr_roi + 0.1 * guess;
    } else {
        guess = CmPoint64f(0,0,0);
    }

    return !bad_frame;
}

///
///
///
void Trackball::updateSphere()
{
    double* m = reinterpret_cast<double*>(_R_roi.data); // absolute orientation (3d mat) in ROI frame

    if (_do_display) {
        _sphere_view.setTo(Scalar::all(128));
    }

    double p2s[3];
    int cnt = 0, good = 0;
    int px = 0, py = 0;
    for (int i = 0; i < _roi_h; i++) {
        const uint8_t* pmask = _roi_mask.ptr(i);
        const uint8_t* proi = _roi_frame.ptr(i);
        for (int j = 0; j < _roi_w; j++) {
            if (pmask[j] < 255) { continue; }
            cnt++;

            // rotate point about rotation axis (sphere coords)
            double* v = &_p1s_lut[(i * _roi_w + j) * 3];
            //p2s[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
            //p2s[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
            //p2s[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
            // transpose - see Localiser::testRotation()
            p2s[0] = m[0] * v[0] + m[3] * v[1] + m[6] * v[2];
            p2s[1] = m[1] * v[0] + m[4] * v[1] + m[7] * v[2];
            p2s[2] = m[2] * v[0] + m[5] * v[1] + m[8] * v[2];


            // map vector in sphere coords to pixel
            if (!_sphere_model->vectorToPixelIndex(p2s, px, py)) { continue; }
            uint8_t& map = _sphere_map.data[py * _sphere_map.step + px];

            // update map tile
            if ((map == 0) || (map == 255)) {
                // map tile frozen
                good++;
            } else if (map == 128) {
                // map tile previously unseen
                map = (proi[j] == 255) ? (128 + SPHERE_MAP_FIRST_HIT_BONUS) : (128 - SPHERE_MAP_FIRST_HIT_BONUS);
            } else {
                good++;
                map = (proi[j] == 255) ? (map + 1) : (map - 1);
            }

            // display
            if (_do_display) { _sphere_view.at<uint8_t>(py, px) = proi[j]; }
        }
    }
    
    if (cnt > 0) {
        _clean_map = false;
        LOG_DBG("Sphere ROI match overlap: %.1f%%", 100 * good / static_cast<double>(cnt));
    }
    else {
        LOG_DBG("Sphere ROI match overlap: 0%%");
    }
}

///
///
///
void Trackball::updatePath()
{
    // rel vec roi
    // _dr_roi

    // abs mat roi
    // _R_roi

    // abs vec roi
    _r_roi = CmPoint64f::matrixToOmega(_R_roi);
    
    // rel vec cam
    _dr_cam = _dr_roi.getTransformed(_roi_to_cam_R);

    // abs mat cam
    _R_cam = _roi_to_cam_R * _R_roi;

    // abs vec cam
    _r_cam = CmPoint64f::matrixToOmega(_R_cam);

    // rel vec world
    _dr_lab = _dr_cam.getTransformed(_cam_to_lab_R);

    // abs mat world
    _R_lab = _cam_to_lab_R * _R_cam;

    // abs vec world
    _r_lab = CmPoint64f::matrixToOmega(_R_lab);


    //// store initial rotation from template (if any)
    //static double Rf[9] = { 1,0,0,0,1,0,0,0,1 };
    //if (first_good_frame) {
    //    memcpy(Rf, Rcam, 9 * sizeof(double));
    //    Maths::MAT_T(Rf);

    //    // clear rel vec cam
    //    r[0] = r[1] = r[2] = 0;

    //    first_good_frame = false;
    //}


    // Rc - abs mat cam (corrected for initial orientation)
    //double Rc[9] = { 1,0,0,0,1,0,0,0,1 };
    //Maths::MUL_MAT(Rf, Rcam, Rc);


    //// FIXME: hack to avoid bee pos history drawing failing
    //// (path hist on ball was failing when Rf != 1)
    //double Rv[3] = { 0,0,0 };
    //Maths::ANGLE_AXIS_FROM_MAT(Rcam, Rv);


    // running speed, radians/frame (-ve rotation around x-axis causes y-axis translation & vice-versa!!)
    _velx = _dr_lab[1];
    _vely = -_dr_lab[0];
    _step_mag = sqrt(_velx * _velx + _vely * _vely); // magnitude (radians) of ball rotation excluding turning (change in heading)
    
    // test data
    if (_cnt > 0) {
        _dist += _step_mag;
        double v = _dr_lab.len();
        double delta = v - _step_avg;
        _step_avg += delta / static_cast<double>(_cnt); // running average
        double delta2 = v - _step_avg;
        _step_var += delta * delta2;                    // running variance (Welford's alg)
    }

    // running direction
    _step_dir = atan2(_vely, _velx);
    if (_step_dir < 0) { _step_dir += 360 * CM_D2R; }

    // integrated x/y pos (optical mouse style)
    _intx += _velx;
    _inty += _vely;

    // integrate bee heading
    _heading -= _dr_lab[2];
    while (_heading < 0) { _heading += 360 * CM_D2R; }
    while (_heading >= 360 * CM_D2R) { _heading -= 360 * CM_D2R; }
    _ang_dist += abs(_dr_lab[2]);

    // integrate 2d position
    {
        const int steps = 4;	// increasing this doesn't help much
        double step = _step_mag / steps;
        static double prev_heading = 0;
        if (_reset) { prev_heading = 0; }
        double heading_step = (_heading - prev_heading);
        while (heading_step >= 180 * CM_D2R) { heading_step -= 360 * CM_D2R; }
        while (heading_step < -180 * CM_D2R) { heading_step += 360 * CM_D2R; }
        heading_step /= steps;  // do after wrapping above

        // super-res integration
        CmPoint64f dir(_velx, _vely, 0);
        dir.normalise();
        dir.rotateAboutNorm(CmPoint(0, 0, 1), prev_heading + heading_step / 2.0);
        for (int i = 0; i < steps; i++) {
            _posx += step * dir[0];
            _posy += step * dir[1];
            dir.rotateAboutNorm(CmPoint(0, 0, 1), heading_step);
        }
        prev_heading = _heading;
    }

    if (_do_display) {
        // update pos hist (in ROI-space!)
        _R_roi_hist.push_back(_R_roi.clone());
        while (_R_roi_hist.size() > DRAW_SPHERE_HIST_LENGTH) {
            _R_roi_hist.pop_front();
        }
        _pos_heading_hist.push_back(CmPoint(_posx, _posy, _heading));
        while (_pos_heading_hist.size() > DRAW_FICTIVE_PATH_LENGTH) {
            _pos_heading_hist.pop_front();
        }
    }
}

///
///
///
bool Trackball::logData()
{
    std::stringstream ss;
    ss.precision(14);

    // frame_count
    ss << _cnt << ", ";
    // rel_vec_cam[3] | error
    ss << _dr_cam[0] << ", " << _dr_cam[1] << ", " << _dr_cam[2] << ", " << _err << ", ";
    // rel_vec_world[3]
    ss << _dr_lab[0] << ", " << _dr_lab[1] << ", " << _dr_lab[2] << ", ";
    // abs_vec_cam[3]
    ss << _r_cam[0] << ", " << _r_cam[1] << ", " << _r_cam[2] << ", ";
    // abs_vec_world[3]
    ss << _r_lab[0] << ", " << _r_lab[1] << ", " << _r_lab[2] << ", ";
    // integrated xpos | integrated ypos | integrated heading
    ss << _posx << ", " << _posy << ", " << _heading << ", ";
    // direction (radians) | speed (radians/frame)
    ss << _step_dir << ", " << _step_mag << ", ";
    // integrated x movement | integrated y movement (mouse output equivalent)
    ss << _intx << ", " << _inty << ", ";
    // timestamp | sequence number
    ss << _ts << ", " << _seq << std::endl;

    // async i/o
    bool ret = true;
    if (_do_sock_output) {
        ret &= _data_sock->addMsg("FT, " + ss.str());
    }
    ret &= _data_log->addMsg(ss.str());
    return ret;
}

///
///
///
double Trackball::testRotation(const double x[3])
{
    static double lmat[9];
    CmPoint64f tmp(x[0], x[1], x[2]);
    tmp.omegaToMatrix(lmat);                // relative rotation in camera frame
    double* rmat = (double*)_R_roi.data;    // pre-multiply to orientation matrix
    static double m[9];                     // absolute orientation in camera frame

    m[0] = lmat[0] * rmat[0] + lmat[1] * rmat[3] + lmat[2] * rmat[6];
    m[1] = lmat[0] * rmat[1] + lmat[1] * rmat[4] + lmat[2] * rmat[7];
    m[2] = lmat[0] * rmat[2] + lmat[1] * rmat[5] + lmat[2] * rmat[8];

    m[3] = lmat[3] * rmat[0] + lmat[4] * rmat[3] + lmat[5] * rmat[6];
    m[4] = lmat[3] * rmat[1] + lmat[4] * rmat[4] + lmat[5] * rmat[7];
    m[5] = lmat[3] * rmat[2] + lmat[4] * rmat[5] + lmat[5] * rmat[8];

    m[6] = lmat[6] * rmat[0] + lmat[7] * rmat[3] + lmat[8] * rmat[6];
    m[7] = lmat[6] * rmat[1] + lmat[7] * rmat[4] + lmat[8] * rmat[7];
    m[8] = lmat[6] * rmat[2] + lmat[7] * rmat[5] + lmat[8] * rmat[8];

    /* Note:
    
    The orientation matrix, _R_roi, is accumulated by pre-multiplying each successive rotation, x.

    When rotating the view vectors, the orientation matrix is also pre-multiplied,
    such that the history of rotations is applied in order.

    See here for explanation of pre- vs post-multiplying:
    http://www.me.unm.edu/~starr/teaching/me582/postmultiply.pdf

    The orientation matrix transpose is used below to rotate the vectors and not the axes.
    */

    double err = 0;
    double p2s[3];
    int cnt = 0, good = 0;
    int px = 0, py = 0;
    for (int i = 0; i < _roi_h; i++) {
        uint8_t* pmask = _roi_mask.ptr(i);
        uint8_t* proi = _roi_frame.ptr(i);
        double* v = &_p1s_lut[i * _roi_w * 3];
        for (int j = 0; j < _roi_w; j++) {
            if (pmask[j] < 255) { continue; }
            cnt++;

            // rotate point about rotation axis (sphere coords)
            int jjj = 3 * j;
            //p2s[0] = m[0] * v[jjj + 0] + m[1] * v[jjj + 1] + m[2] * v[jjj + 2];
            //p2s[1] = m[3] * v[jjj + 0] + m[4] * v[jjj + 1] + m[5] * v[jjj + 2];
            //p2s[2] = m[6] * v[jjj + 0] + m[7] * v[jjj + 1] + m[8] * v[jjj + 2];
            // transpose
            p2s[0] = m[0] * v[jjj + 0] + m[3] * v[jjj + 1] + m[6] * v[jjj + 2];
            p2s[1] = m[1] * v[jjj + 0] + m[4] * v[jjj + 1] + m[7] * v[jjj + 2];
            p2s[2] = m[2] * v[jjj + 0] + m[5] * v[jjj + 1] + m[8] * v[jjj + 2];

            // map vector in sphere coords to pixel
            if (!_sphere_model->vectorToPixelIndex(p2s, px, py)) { continue; }  // sphere model is spherical, so pixel should never fall outside valid area

            int r = proi[j];
            int s = _sphere_map.data[py * _sphere_map.step + px];
            if (s == 128) { continue; }
            err += (r - s) * (r - s);
            good++;     // number of test pixels that correspond to previously seen pixels
        }
    }

    /// Compute avg squared diff error.
    if ((cnt > 0) && (good / static_cast<double>(cnt) > 0.25)) {
        err /= good;
    } else {
        err = DBL_MAX;
    }

    return err;
}

///
///
///
void makeSphereRotMaps(
    CameraModelPtr cam_model,
    Mat& mapX, Mat& mapY, const Mat& mask,
    double sphere_r_d_ratio, const CmPoint64f& rot_angle_axis)
{
    CmPoint64f sc(0, 0, 1);
    int w = mapX.cols, h = mapX.rows;
    float *mapx = (float*)mapX.data, *mapy = (float*)mapY.data;
    for (int i = 0, it = 0; i < h; ++i) {
        const uint8_t* pmask = mask.ptr(i);
        for (int j = 0; j < w; ++j, ++it) {
            if (pmask[j] < 255) {
                mapx[it] = -1;
                mapy[it] = -1;
                continue;
            }

            CmPoint64f p;
            cam_model->pixelIndexToVector(j, i, p);
            p.normalise();

            double a = 1.0;
            double b = 2.0*(p[0] * -sc[0] + p[1] * -sc[1] + p[2] * -sc[2]);
            double c = sc[0] * sc[0] + sc[1] * sc[1] + sc[2] * sc[2] - sphere_r_d_ratio * sphere_r_d_ratio;

            double quad = b * b - 4 * a*c;
            if (quad < 0) {
                mapx[it] = -1;
                mapy[it] = -1;
                continue;
            }

            // get point on surface of sphere (camera coords)
            // double u1 = (-b+sqrt(quad))/(2.0*a);
            double u2 = (-b - sqrt(quad)) / (2.0 * a);
            // double u = std::min(u1,u2);
            //CmPoint64f p1 = p * u2;

            // switch to sphere coords
            p = u2 * p - sc;

            // rotate point about rotation axis (sphere coords)
            // -ve rotation to rotate vector, not axes (see Localiser::testRotation())
            p.rotateAbout(-rot_angle_axis);

            // check whether point has disappeared
            if (p.dot(sc) > 0) {
                mapx[it] = -1;
                mapy[it] = -1;
                continue;
            }

            // switch back to camera coords
            p = p + sc;

            // re-intersect with camera model
            double x2 = 0, y2 = 0;
            cam_model->vectorToPixelIndex(p, x2, y2);

            mapx[it] = static_cast<float>(x2);
            mapy[it] = static_cast<float>(y2);
        }
    }
}



///
///
///
bool Trackball::updateCanvasAsync(std::shared_ptr<DrawData> data)
{
    bool ret = false;
    lock_guard<mutex> l(_drawMutex);
    if (_active) {
        _drawQ.push_back(data);
        _drawCond.notify_all();
        ret = true;
    }
    return ret;
}

///
///
///
void Trackball::processDrawQ()
{
    /// Set thread higher priority (when run as SU).
    if (!SetThreadNormalPriority()) {
        LOG_ERR("Error! Unable to set thread priority!");
    }

    /// Get a un/lockable lock.
    unique_lock<mutex> l(_drawMutex);

    /// Process drawing queue.
    while (_active) {
        /// Wait for data.
        while (_drawQ.size() == 0) {
            _drawCond.wait(l);
            if (!_active) { break; }
        }
        if (!_active) { break; }

        /// Retrieve data.
        auto data = _drawQ.back();

        /// Clear all other frames (only draw latest available).
        if (_drawQ.size() > 1) {
            LOG_DBG("Skipping drawing %d frames.", _drawQ.size() - 1);
        }
        _drawQ.clear();

        l.unlock();

        /// Draw canvas unlocked.
        drawCanvas(data);

        l.lock();
    }
    l.unlock();

    LOG_DBG("Finished processing drawing queue.");
}

///
///
///
void Trackball::drawCanvas(std::shared_ptr<DrawData> data)
{
    static Mat canvas(3 * DRAW_CELL_DIM, 4 * DRAW_CELL_DIM, CV_8UC3);
    canvas.setTo(Scalar::all(0));

    /// Unpack current data.
    Mat& src_frame = data->src_frame;
    Mat& roi_frame = data->roi_frame;
    CmPoint64f& dr_roi = data->dr_roi;
    Mat& R_roi = data->R_roi;
    Mat& sphere_view = data->sphere_view;
    Mat& sphere_map = data->sphere_map;
    deque<Mat>& R_roi_hist = data->R_roi_hist;
    deque<CmPoint64f>& pos_heading_hist = data->pos_heading_hist;

    /// Draw source image.
    double radPerPix = _sphere_rad * 3.0 / (2 * DRAW_CELL_DIM);
    static CameraModelPtr draw_camera = CameraModel::createFisheye(
        2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, radPerPix, 360 * CM_D2R);
    static CameraRemapPtr draw_remapper = CameraRemapPtr(new CameraRemap(
        _src_model, draw_camera, _cam_to_roi));
        
    Mat draw_input = canvas(Rect(0, 0, 2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM));
    draw_remapper->apply(src_frame, draw_input);

    /// Sphere warping.
    static Mat mapX(_roi_h, _roi_w, CV_32FC1);
    mapX.setTo(Scalar::all(-1));
    static Mat mapY(_roi_h, _roi_w, CV_32FC1);
    mapY.setTo(Scalar::all(-1));
    makeSphereRotMaps(_roi_model, mapX, mapY, _roi_mask, _r_d_ratio, dr_roi);

    BasicRemapper warper(_roi_w, _roi_h, mapX, mapY);
    static Mat prev_roi = roi_frame;   // no copy!
    static Mat warp_roi(_roi_h, _roi_w, CV_8UC1);
    warp_roi.setTo(Scalar::all(0));
    warper.apply(prev_roi, warp_roi);
    prev_roi = roi_frame;  // no copy!

    /// Diff image.
    static Mat diff_roi(_roi_h, _roi_w, CV_8UC1);
    cv::absdiff(roi_frame, warp_roi, diff_roi);

    /// Draw thresholded ROI.
    static Mat resize_roi(DRAW_CELL_DIM, DRAW_CELL_DIM, CV_8UC1);
    cv::resize(roi_frame, resize_roi, resize_roi.size());
    Mat draw_roi = canvas(Rect(2 * DRAW_CELL_DIM, 0, DRAW_CELL_DIM, DRAW_CELL_DIM));
    cv::cvtColor(resize_roi, draw_roi, cv::COLOR_GRAY2BGR);

    /// Draw warped diff ROI.
    static Mat resize_diff(DRAW_CELL_DIM, DRAW_CELL_DIM, CV_8UC1);
    cv::resize(diff_roi, resize_diff, resize_diff.size());
    Mat draw_diff = canvas(Rect(3 * DRAW_CELL_DIM, 0, DRAW_CELL_DIM, DRAW_CELL_DIM));
    cv::cvtColor(resize_diff, draw_diff, cv::COLOR_GRAY2BGR);

    /// Draw current sphere view.
    static Mat resize_view(DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, CV_8UC1);
    cv::resize(sphere_view, resize_view, resize_view.size());
    Mat draw_view = canvas(Rect(2 * DRAW_CELL_DIM, 1 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, DRAW_CELL_DIM));
    cv::cvtColor(resize_view, draw_view, cv::COLOR_GRAY2BGR);

    /// Draw current sphere map.
    static Mat resize_map(DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, CV_8UC1);
    cv::resize(sphere_map, resize_map, resize_map.size());
    Mat draw_map = canvas(Rect(2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, DRAW_CELL_DIM));
    cv::cvtColor(resize_map, draw_map, cv::COLOR_GRAY2BGR);

    /// Draw fictive path.
    //FIXME: add heading arrow to fictive path
    {
        int npts = pos_heading_hist.size();
        double minx = 0, maxx = 0, miny = 0, maxy = 0;
        for (int i = 0; i < npts; i++) {
            double x = pos_heading_hist[i].x, y = pos_heading_hist[i].y;
            if (x < minx)
                minx = x;
            if (x > maxx)
                maxx = x;
            if (y < miny)
                miny = y;
            if (y > maxy)
                maxy = y;
        }
        double scl = 1;
        if (npts > 1) {
            double sclx = double(DRAW_CELL_DIM - 8) / (2.0 * std::max(fabs(minx), fabs(maxx)));
            double scly = double(DRAW_CELL_DIM - 4) / std::max(fabs(miny), fabs(maxy));
            scl = std::min(sclx, scly);
        }

        Mat draw_path = canvas(Rect(0, 2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, DRAW_CELL_DIM));
        double cx = DRAW_CELL_DIM, cy = 0.5 * DRAW_CELL_DIM;
        double ppx = cx, ppy = cy;
        for (int i = 0; i < npts; i++) {
            double px = cx + scl * pos_heading_hist[i].y, py = cy - scl * pos_heading_hist[i].x;
            cv::line(draw_path,
                cv::Point(static_cast<int>(round(ppx * 16)), static_cast<int>(round(ppy * 16))),
                cv::Point(static_cast<int>(round(px * 16)), static_cast<int>(round(py * 16))),
                CV_RGB(255, 255, 255), 1, cv::LINE_AA, 4);
            ppx = px;
            ppy = py;
        }
    }

    /// Draw sphere orientation history (animal position history on sphere).
    {
        static const CmPoint up(0, 0, -1.0);
        static CmPoint up_roi = up.getTransformed(_roi_to_cam_R.t() * _cam_to_lab_R.t()).getNormalised() * _r_d_ratio;

        double ppx = -1, ppy = -1;
        draw_camera->vectorToPixelIndex(up_roi, ppx, ppy);
        for (int i = R_roi_hist.size() - 1; i >= 0; i--) {
            // multiply by transpose - see Localiser::testRotation()
            CmPoint vec = up_roi.getTransformed(R_roi * R_roi_hist[i].t()).getNormalised() * _r_d_ratio;
                
            // sphere is centred at (0,0,1) cam coords, with r
            double px = -1, py = -1;

            // don't draw link if current point is on back side of sphere
            if (vec[2] < 0) {
                vec[2] += 1;    // switch from sphere to cam coords
                draw_camera->vectorToPixelIndex(vec, px, py);

                // draw link
                if ((ppx >= 0) && (ppy >= 0) && (px >= 0) && (py >= 0) && (ppx < draw_input.cols) && (ppy > draw_input.rows) && (px < draw_input.cols) && (py < draw_input.rows)) {
                    float mix = (i + 0.5f) / static_cast<float>(R_roi_hist.size());
                    cv::Vec3b rgb = draw_input.at<cv::Vec3b>(static_cast<int>((ppy + py) / 2.f), static_cast<int>((ppx + px) / 2.f));   // px/py are pixel index values
                    int b = static_cast<int>((1 - mix) * rgb[0] + mix * 255.f + 0.5f);
                    int g = static_cast<int>((1 - mix) * rgb[1] + mix * 255.f + 0.5f);
                    int r = static_cast<int>((1 - mix) * rgb[2] + mix * 0.f + 0.5f);

                    cv::line(draw_input,
                        cv::Point(static_cast<int>(round(px * 16)), static_cast<int>(round(py * 16))),
                        cv::Point(static_cast<int>(round(ppx * 16)), static_cast<int>(round(ppy * 16))),
                        CV_RGB(r, g, b), 1, cv::LINE_AA, 4);
                }
            }
            ppx = px;
            ppy = py;
        }
    }

    /// Draw lines.
    cv::line(canvas,
        cv::Point(2 * DRAW_CELL_DIM, 0 * DRAW_CELL_DIM) * 16,
        cv::Point(2 * DRAW_CELL_DIM, 3 * DRAW_CELL_DIM) * 16,
        CV_RGB(255, 255, 255), 2, cv::LINE_AA, 4);
    cv::line(canvas,
        cv::Point(0 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM) * 16,
        cv::Point(4 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM) * 16,
        CV_RGB(255, 255, 255), 2, cv::LINE_AA, 4);
    cv::line(canvas,
        cv::Point(3 * DRAW_CELL_DIM, 0 * DRAW_CELL_DIM) * 16,
        cv::Point(3 * DRAW_CELL_DIM, 1 * DRAW_CELL_DIM) * 16,
        CV_RGB(255, 255, 255), 2, cv::LINE_AA, 4);
    cv::line(canvas,
        cv::Point(2 * DRAW_CELL_DIM, 1 * DRAW_CELL_DIM) * 16,
        cv::Point(4 * DRAW_CELL_DIM, 1 * DRAW_CELL_DIM) * 16,
        CV_RGB(255, 255, 255), 2, cv::LINE_AA, 4);

    /// Draw text (with shadow).
    shadowText(canvas, string("Processed ") + dateString(),
        2, 15,
        255, 255, 0);
    shadowText(canvas, string("FicTrac (") + string(__DATE__) + string(")"),
        canvas.cols - 184, 15,
        255, 255, 0);
    shadowText(canvas, "input image",
        2, 2 * DRAW_CELL_DIM - 8, 
        255, 255, 0);
    shadowText(canvas, "flat path",
        2, 3 * DRAW_CELL_DIM - 8,
        255, 255, 0);
    shadowText(canvas, "accumulated map",
        2 * DRAW_CELL_DIM + 3, 3 * DRAW_CELL_DIM - 8,
        255, 255, 0);
    shadowText(canvas, "sphere ROI",
        2 * DRAW_CELL_DIM + 3, 1 * DRAW_CELL_DIM - 8,
        255, 255, 0);
    shadowText(canvas, "instant map",
        2 * DRAW_CELL_DIM + 3, 2 * DRAW_CELL_DIM - 8,
        255, 255, 0);
    shadowText(canvas, "warped diff",
        3 * DRAW_CELL_DIM + 3, 1 * DRAW_CELL_DIM - 8,
        255, 255, 0);

    /// Display
    cv::imshow("FicTrac-debug", canvas);
    uint16_t key = cv::waitKey(1);
    if (key == 0x1B) {  // esc
        LOG("Exiting..");
        terminate();
    }

    if (_save_raw) {
        _raw_vid.write(src_frame);
    }
    if (_save_debug) {
        _debug_vid.write(canvas);
    }
}

void Trackball::printState()
{
    PRINT("");
    PRINT("Trackball state");
    PRINT("Sphere orientation (cam): %f %f %f", _r_cam[0], _r_cam[1], _r_cam[2]);
    PRINT("Total heading rotation: %f deg", _ang_dist * CM_R2D);
    PRINT("Heading direction: %f deg (%f %% total heading rotation)", _heading * CM_R2D, _heading * 100. / _ang_dist);
    PRINT("Accumulated X/Y motion: %f / %f rad (%f / %f * 2pi)", _intx, _inty, _intx / (2 * CM_PI), _inty / (2 * CM_PI));
    PRINT("Distance travelled: %f rad (%f * 2pi)", _dist, _dist / (2 * CM_PI));
    PRINT("Integrated X/Y position: (%.3e, %.3e) rad (%f / %f %% total path length)", _posx, _posy, _posx * 100. / _dist, _posy * 100. / _dist);
    PRINT("Average/stdev rotation: %.3e / %.3e rad/frame", _step_avg, sqrt(_step_var / _cnt));  // population variance
}

bool Trackball::writeTemplate(std::string fn)
{
    if (!_init) { return false; }
    
    string template_fn = _base_fn + "-template.png";

    bool ret = cv::imwrite(template_fn, _sphere_map);
    if (!ret) {
        LOG_ERR("Error! Could not write template to disk (%s).", template_fn.c_str());
    } else {
        LOG("Template saved to disk (%s).", template_fn.c_str());
    }
    return ret;
}
