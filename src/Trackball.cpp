/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Trackball.cpp
/// \brief      Stores surface map and current orientation of tracking ball.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//FIXME: better alg for preferancing high-overlap good matches?

#include "Trackball.h"

#include "geometry.h"
#include "Logger.h"
#include "timing.h"
#include "CameraRemap.h"
#include "BasicRemapper.h"
#include "CVSource.h"

#include <sstream>
#include <cmath>

using std::string;
using std::unique_ptr;
using std::shared_ptr;
using std::unique_lock;
using std::mutex;
using std::vector;
using cv::Mat;
using cv::Scalar;
using cv::Rect;
using cv::Point2d;

const int POS_HIST_LENGTH = 250;
const int DRAW_CELL_DIM = 160;
const int FICTIVE_DRAW_POINTS = 1000;

const int QUALITY_DEFAULT = 6;
const double OPT_TOL_DEFAULT = 1e-4;
const double OPT_BOUND_DEFAULT = 0.5;
const int OPT_MAX_EVAL_DEFAULT = 100;
const bool OPT_GLOBAL_SEARCH_DEFAULT = false;
const int OPT_MAX_BAD_FRAMES_DEFAULT = -1;

const bool DO_DISPLAY_DEFAULT = true;

///
///
///
bool intersectSphere(double camVec[3], double sphereVec[3], double r)
{
    double q = camVec[2] * camVec[2] + r*r;

    if (q < 1) { return false; }

    // get point on front surface of sphere (camera coords)
    double u = camVec[2] - sqrt(q - 1);

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
    : _active(false), _reset(true)
{
    /// Load and parse config file.
    if (_cfg.read(cfg_fn) <= 0) {
        LOG_ERR("Error parsing config file (%s)!", cfg_fn.c_str());
        return;
    }

    /// Open frame source.
    string source_fn = _cfg("src_fn");
    //FIXME: support multiple frame sources
    shared_ptr<CVSource> source = shared_ptr<CVSource>(new CVSource(source_fn));
    if (!source->isOpen()) {
        LOG_ERR("Error! Could not open input frame source (%s)!", source_fn.c_str());
        return;
    }

    /// Create source mask from ignore region.
    //string mask_fn = _cfg("mask_fn");
    //if (mask_fn.empty() && !source->isLive()) {
    //    // default to source_fn-mask.jpg, if we're not running live from camera
    //    mask_fn = source_fn.substr(0, source_fn.length() - 4) + "-mask.jpg";
    //}
    Mat source_mask;// = cv::imread(mask_fn, 0);
    //if (source_mask.empty()) {
    //    LOG_ERR("Error reading source mask image (%s)!", mask_fn.c_str());
    //    return;
    //}
    //else if ((source_mask.cols != source->getWidth()) || (source_mask.rows != source->getHeight())) {
    //    LOG_ERR("Error! Source mask image dimensions (%dx%d) do not match source (%dx%d).", source_mask.cols, source_mask.rows, source->getWidth(), source->getHeight());
    //    return;
    //}

    /// Source camera model.
    double vfov = -1;
    if (!_cfg.getDbl("vfov", vfov) || vfov <= 0) {
        LOG_ERR("Error! Camera vertical FoV parameter specified in the config file (vfov) is invalid!");
        return;
    }
    //FIXME: support other camera models
    _src_model = CameraModel::createRectilinear(source->getWidth(), source->getHeight(), vfov);

    /// Dimensions - quality defaults to 6 (remap_dim 60x60, sphere_dim 180x90).
    int quality_factor = QUALITY_DEFAULT;
    if (!_cfg.getInt("q_factor", quality_factor)) {
        LOG_WRN("Warning! Quality parameter specified in the config file (q_factor) is invalid! Using default value (%d).", quality_factor);
    }
    _roi_w = _roi_h = 10 * quality_factor;
    _map_h = static_cast<int>(1.5 * _roi_h);
    _map_w = 2 * _map_h;

    /// Load sphere circumference points and fit c, r.
    {
        // read pts from config file
        bool pass = false;
        vector<int> circ_pxs;
        if (_cfg.getVecInt("roi_circ", circ_pxs)) {
            vector<Point2d> circ_pts;
            for (unsigned int i = 1; i < circ_pxs.size(); i += 2) {
                circ_pts.push_back(Point2d(circ_pxs[i - 1], circ_pxs[i]));
            }

            // fit circular fov
            if ((circ_pts.size() >= 3) && circleFit_camModel(circ_pts, _src_model, _sphere_c, _sphere_fov)) {
                // NOTE: sin rather than tan to correct for apparent size of sphere
                _r_d_ratio = sin(_sphere_fov / 2.0);
                pass = true;
            }
        }
        if (!pass) {
            LOG_ERR("Error! Sphere circumference points specified in config file (roi_circ) are invalid!");
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

        // Cam to lab transformation from configuration.
        vector<double> r_c2a;
        if (_cfg.getVecDbl("r_c2a", r_c2a) && (r_c2a.size() == 3)) {
            cam_to_lab_r = CmPoint64f(r_c2a[0], r_c2a[1], r_c2a[2]);
            _cam_to_lab_R = CmPoint64f::omegaToMatrix(cam_to_lab_r);
        }
        else {
            LOG_ERR("Error! Camera-to-lab coordinate tranformation specified in config file (r_c2a) is invalid!");
            return;
        }
    }

    ///// Remap (ROI) model and remapper.
    double sphere_radPerPix = _sphere_fov / _roi_w;
    CameraModelPtr roi_model = CameraModel::createFisheye(_roi_w, _roi_h, sphere_radPerPix, _sphere_fov);
    _cam_to_roi = MatrixRemapTransform::createFromOmega(-roi_to_cam_r);
    CameraRemapPtr remapper = CameraRemapPtr(new CameraRemap(_src_model, roi_model, _cam_to_roi));

    /// ROI mask.
    _roi_mask.create(_roi_h, _roi_w, CV_8UC1);
    _roi_mask.setTo(cv::Scalar::all(255));
    remapper->apply(source_mask, _roi_mask);
    erode(_roi_mask, _roi_mask, Mat(), cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);   // remove edge effects

     /// Frame source.
    double thresh_ratio, thresh_win_pc;
    if (!_cfg.getDbl("thr_ratio", thresh_ratio) || !_cfg.getDbl("thr_win_pc", thresh_win_pc)) {
        LOG_ERR("Error! Thresholding parameters specified in the config file (thr_ratio, thr_win_pc) are invalid!");
        return;
    }
    _frameGrabber = unique_ptr<FrameGrabber>(new FrameGrabber(
        source,
        remapper,
        _roi_mask,
        thresh_ratio,
        thresh_win_pc
    ));

    /// Init optimiser.
    double tol = OPT_TOL_DEFAULT;
    if (!_cfg.getDbl("opt_tol", tol)) {
        LOG_WRN("Warning! Optimisation parameter specified in the config file (opt_tol) is invalid! Using default value (%f).", tol);
    }
    _bound = OPT_BOUND_DEFAULT;
    if (!_cfg.getDbl("opt_bound", _bound)) {
        LOG_WRN("Warning! Optimisation parameter specified in the config file (opt_bound) is invalid! Using default value (%f).", _bound);
    }
    int max_evals = OPT_MAX_EVAL_DEFAULT;
    if (!_cfg.getInt("opt_max_evals", max_evals)) {
        LOG_WRN("Warning! Optimisation parameter specified in the config file (opt_max_eval) is invalid! Using default value (%d).", max_evals);
    }
    _global_search = OPT_GLOBAL_SEARCH_DEFAULT;
    if (!_cfg.getBool("opt_do_global", _global_search)) {
        LOG_WRN("Warning! Optimisation parameter specified in the config file (opt_do_global) is invalid! Using default value (%d).", _global_search);
    }
    _max_bad_frames = OPT_MAX_BAD_FRAMES_DEFAULT;
    if (!_cfg.getInt("max_bad_frames", _max_bad_frames)) {
        LOG_WRN("Warning! Optimisation parameter specified in the config file (max_bad_frames) is invalid! Using default value (%d).", _max_bad_frames);
    }
    _error_thresh = -1;
    if (!_cfg.getDbl("opt_max_err", _error_thresh) || _error_thresh < 0) {
        LOG_ERR("Error! Optimisation parameter specified in the config file (opt_max_err) is invalid!");
        return;
    }
    init(NLOPT_LN_BOBYQA, 3);
    setLowerBounds(-_bound);
    setUpperBounds(_bound);
    setFtol(tol);
    setXtol(tol);
    setMaxEval(max_evals);

    /// Surface mapping.
    _sphere_model = CameraModel::createEquiArea(_map_w, _map_h);

    /// Buffers.
    _sphere.create(_map_h, _map_w, CV_8UC1);
    _sphere_max.create(_map_h, _map_w, CV_32SC1);
    _sphere_hist.create(_map_h, _map_w * 3, CV_32SC1);	// hist stores histogram for 3 classes - black, grey, white

    /// Surface map template.
    Mat sphere_template;
    {
        string sphere_template_fn = _cfg("sphere_map_fn");
        sphere_template = cv::imread(sphere_template_fn, 0);
        if ((sphere_template.cols != _map_w) || (sphere_template.rows != _map_h)) {
            LOG_ERR("Error! Sphere map template specified in the config file (sphere_map_fn) is invalid (%dx%d)!", sphere_template.cols, sphere_template.rows);
            return;
        }
    }
    if (!sphere_template.empty() && (sphere_template.size() == _sphere.size())) {
        sphere_template.copyTo(_sphere);
        for (int i = 0; i < _map_h; i++) {
            int32_t* pmax = (int32_t*)_sphere_max.ptr(i);
            int32_t* phist = (int32_t*)_sphere_hist.ptr(i);
            uint8_t* psphere = _sphere.ptr(i);
            for (int j = 0; j < _map_w; j++) {
                pmax[j] = 10;   // arbitrary start weight
                phist[j * 3 + static_cast<int>(psphere[j]/127.f)] = pmax[j];	// thresholded sphere is quantised to levels 0 = black, 1 = grey, 2 = white
            }
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
            roi_model->pixelIndexToVector(j, i, l);
            vec3normalise(l);

            double* s = &_p1s_lut[(i * _roi_w + j) * 3];
            if (!intersectSphere(l, s, _r_d_ratio)) { pmask[j] = 128; }
        }
    }

    /// Output.
    string data_file_name = _cfg("output_fn");
    if (data_file_name.empty()) {
        data_file_name = string("fictrac_") + execTime() + ".dat";
    }
    _log = unique_ptr<Recorder>(new Recorder(data_file_name));

    /// Display.
    _do_display = DO_DISPLAY_DEFAULT;
    if (!_cfg.getBool("do_display", _do_display)) {
        LOG_WRN("Warning! Display parameter specified in the config file (do_display) is invalid! Using default value (%d).", _do_display);
    }
    if (_do_display) {
        _view.create(_map_h, _map_w, CV_8UC1);
        _view.setTo(Scalar::all(128));

        _canvas.create(3 * DRAW_CELL_DIM, 4 * DRAW_CELL_DIM, CV_8UC3);
        _canvas.setTo(Scalar::all(0));
    }

    /// Data.
    _cnt = 0;
    _err = 0;
    _intx = _inty = 0;
    reset();

    /// Thread stuff.
    _active = true;
    _t1 = unique_ptr<std::thread>(new std::thread(&Trackball::process, this));
}

///
/// Default destructor.
///
Trackball::~Trackball()
{
    LOG("Closing sphere tracker..");

    _active = false;

    if (_t1 && _t1->joinable()) {
        _t1->join();
    }
}

///
///
///
void Trackball::reset()
{
    _reset = true;

    /// Clear maps.
    _sphere.setTo(Scalar::all(128));
    _sphere_max.setTo(Scalar::all(0));
    _sphere_hist.setTo(Scalar::all(0));

    /// Reset sphere.
    _R_roi = Mat::eye(3, 3, CV_64F);

    /// Reset data.
    _seq = 0;       // indicates new sequence started
    _posx = 0;      // reset because heading was lost
    _posy = 0;
    _heading = 0;

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

    /// Sphere tracking loop.
    int nbad = 0;
    while (_active && _frameGrabber->getNextFrameSet(_src_frame, _roi_frame, _ts)) {

        /// Localise current view of sphere.
        if (!doSearch(_global_search)) {
            LOG_ERR("Error! Could not match current sphere orientation to within error threshold (%d). No data will be output for this frame!", _error_thresh);
            nbad++;
        }
        else {
            updateSphere();
            updatePath();
            logData();  // only output good data
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
            drawCanvas();

            cv::imshow("FicTrac-debug", _canvas);
            uint16_t key = cv::waitKey(1);
            if (key == 0x1B) {  // esc
                LOG("Exiting..");
                _active = false;
            }
        }

        /// Always increment frame counter.
        _cnt++;

        /// Clear reset flag.
        _reset = false;
    }

    LOG_DBG("Stopping sphere tracking loop!");

    _active = false;
}

///
///
///
bool Trackball::doSearch(bool allow_global = false)
{
    /// Maintain a low-pass filtered rotation to use as guess.
    static CmPoint64f guess;
    guess = 0.9 * _dr_roi + 0.1 * guess;

    /// Constrain search to bound around guess.
    double lb[3] = { guess[0] - _bound, guess[1] - _bound, guess[2] - _bound };
    double ub[3] = { guess[0] + _bound, guess[1] + _bound, guess[2] + _bound };
    setLowerBounds(lb);
    setUpperBounds(ub);

    /// Run optimisation and save result.
    double guessv[3];
    guess.copyTo(guessv);
    optimize(guessv);
    getOptX(guessv);
    _dr_roi.copy(guessv);
    _err = getOptF();

    /// Check optimisation.
    bool bad_frame = _error_thresh >= 0 ? (_err > _error_thresh) : false;
    if (allow_global && bad_frame) {    // && SPHERE_INIT?
        // do global search
        //double best_score = err;
        //double best_guess[3] = { guess[0], guess[1], guess[2] };
        //double test[3] = { 0 };
        //int nsearch_pts = 1000;
        //int search_state = 0;
        //printf("performing global search... ");
        //fflush(stdout);
        //for (int i = 1; i <= nsearch_pts; i++) {
        //    float pc = float(i) / nsearch_pts;
        //    switch (search_state) {
        //    case 0:
        //        if (pc >= 0.1) {
        //            printf("10%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 1:
        //        if (pc >= 0.2) {
        //            printf("20%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 2:
        //        if (pc >= 0.3) {
        //            printf("30%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 3:
        //        if (pc >= 0.4) {
        //            printf("40%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 4:
        //        if (pc >= 0.5) {
        //            printf("50%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 5:
        //        if (pc >= 0.6) {
        //            printf("60%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 6:
        //        if (pc >= 0.7) {
        //            printf("70%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 7:
        //        if (pc >= 0.8) {
        //            printf("80%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 8:
        //        if (pc >= 0.9) {
        //            printf("90%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    case 9:
        //        if (pc >= 1) {
        //            printf("100%% ");
        //            fflush(stdout);
        //            search_state++;
        //        }
        //        break;
        //    default:
        //        break;
        //    }
        //    fflush(stdout);

        //    guess[0] = Utils::GEN_RAND_GSN(1, 0);
        //    guess[1] = Utils::GEN_RAND_GSN(1, 0);
        //    guess[2] = Utils::GEN_RAND_GSN(1, 0);
        //    Maths::NORMALISE_VEC(guess);

        //    for (int mag = -180; mag < 180; mag += 10) {
        //        double rmag = mag * Maths::D2R;
        //        test[0] = rmag * guess[0];
        //        test[1] = rmag * guess[1];
        //        test[2] = rmag * guess[2];

        //        err = sphere.testRotation(test);

        //        if (err < best_score) {
        //            best_score = err;
        //            best_guess[0] = test[0];
        //            best_guess[1] = test[1];
        //            best_guess[2] = test[2];
        //        }

        //        //						if( !(bad_frame = err > max_err) ) { break; }
        //    }
        //    //					if( !bad_frame ) { break; }
        //}
        //printf("\nsearch:\t\t%.3f %.3f %.3f  (err=%.3f)\n",
        //    best_guess[0], best_guess[1], best_guess[2], best_score);

        //double lb[3] = { best_guess[0] - nlopt_res, best_guess[1] - nlopt_res, best_guess[2] - nlopt_res };
        //double ub[3] = { best_guess[0] + nlopt_res, best_guess[1] + nlopt_res, best_guess[2] + nlopt_res };
        //sphere.setLowerBounds(lb);
        //sphere.setUpperBounds(ub);
        //sphere.optimize(best_guess);
        //sphere.getOptX(guess);
        //err = sphere.getOptF();

        //bad_frame = err > max_err;

        //printf("minimised:\t\t%.3f %.3f %.3f  (err=%.3f)\n",
        //    guess[0], guess[1], guess[2], err);
    }

    LOG("optimum sphere rotation:\t%.3f %.3f %.3f  (err=%.3f/its=%d)", _dr_roi[0], _dr_roi[1], _dr_roi[2], _err, getNumEval());

    return !bad_frame;
}

///
///
///
void Trackball::updateSphere()
{
    Mat tmpR = CmPoint64f::omegaToMatrix(_dr_roi); // relative rotation (angle-axis) in ROI frame
                                                   // post-multiplication!
    _R_roi = _R_roi * tmpR;                        // absolute orientation (3d mat) in ROI frame

    if (_do_display) {
        _view.setTo(Scalar::all(128));
    }

    Mat p2s(3, 1, CV_64F);
    int cnt = 0, good = 0;
    for (int i = 0; i < _roi_h; i++) {
        uint8_t* pthresh = _roi_frame.ptr(i);
        for (int j = 0; j < _roi_w; j++) {
            if (_roi_mask.at<uint8_t>(i, j) < 255) { continue; }
            cnt++;

            // rotate point about rotation axis (sphere coords)
            Mat p1s(3, 1, CV_64F, &_p1s_lut[(i * _roi_w + j) * 3]);
            p2s = _R_roi * p1s;

            // map vector in sphere coords to pixel
            int x = 0, y = 0;
            if (!_sphere_model->vectorToPixelIndex(reinterpret_cast<double*>(p2s.data), x, y)) { continue; }

            int32_t* smax = &_sphere_max.at<int32_t>(y, x);
            if (*smax > 0) { good++; }

            int h = ++(_sphere_hist.at<int32_t>(y, x * 3 + static_cast<int>(pthresh[j]/127.f)));	// thresholded sphere is quantised to levels 0 = black, 1 = grey, 2 = white
            if (h > *smax) {
                *smax = h;
                _sphere.at<uint8_t>(y, x) = pthresh[j];
            }

            // display
            if (_do_display) { _view.at<uint8_t>(y, x) = pthresh[j]; }
        }
    }

    LOG_DBG("Match overlap: %.1f", 100 * good / static_cast<double>(cnt));
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
    //FIXME: scale by -1?

    // abs mat cam
    _R_cam = _roi_to_cam_R * _R_roi;
    //FIXME: transpose?

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

    // integrate 2d position
    {
        const int steps = 4;	// increasing this doesn't help much
        double step = _step_mag / steps;
        static double prev_heading = 0;
        if (_reset) { prev_heading = 0; }
        double heading_step = (_heading - prev_heading) / steps;
        while (heading_step >= 180 * CM_D2R) { heading_step -= 360 * CM_D2R; }
        while (heading_step < -180 * CM_D2R) { heading_step += 360 * CM_D2R; }

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
        _R_roi_hist.push_back(_R_roi);
        while (_R_roi_hist.size() > POS_HIST_LENGTH) {
            _R_roi_hist.pop_front();
        }

        // Warning! No length limit - will grow at approx 8MB/hour @ 100 Hz.
        _pos_heading_hist.push_back(CmPoint(_posx, _posy, _heading));
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
    ss << _ts << ", " << _seq << ", ";

    // async i/o
    return _log->addMsg(ss.str());
}

///
///
///
double Trackball::testRotation(const double x[3])
{
    Mat tmpR = CmPoint64f::omegaToMatrix(CmPoint64f(x[0], x[1], x[2])); // relative rotation in camera frame
                                                                        // post-multiplication!
    Mat testR = _R_roi * tmpR;                                          // absolute orientation in camera frame

    double err = 0;
    Mat p2s(3, 1, CV_64F);
    int cnt = 0, good = 0;
    for (int i = 0; i < _roi_h; i++) {
        for (int j = 0; j < _roi_w; j++) {
            if (_roi_mask.at<uint8_t>(i, j) < 255) { continue; }
            cnt++;

            // rotate point about rotation axis (sphere coords)
            Mat p1s(3, 1, CV_64F, &_p1s_lut[(i * _roi_w + j) * 3]);
            p2s = testR * p1s;

            // map vector in sphere coords to pixel
            int x = 0, y = 0;
            if (!_sphere_model->vectorToPixelIndex(reinterpret_cast<double*>(p2s.data), x, y)) { continue; }

            if (_sphere_max.at<int32_t>(y, x) == 0) { continue; }
            int px = _roi_frame.at<uint8_t>(i, j);
            if (px == 128) { continue; }
            double diff = (px - static_cast<int>(_sphere.at<uint8_t>(y, x)));
            err += diff*diff;
            good++;
        }
    }

    /// Normalise for percentage overlap (avoid false positive good matches with little overlap)
    if (good > 0) {
        err *= cnt / static_cast<double>(good);
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
    double sphere_r_d_ratio, CmPoint64f& rot_angle_axis)
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
            p.rotateAbout(rot_angle_axis);

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
void shadowText(Mat& img, string text, int px, int py, int r, int g, int b)
{
    cv::putText(img, text, cv::Point(px + 1, py + 1), CV_FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0, 0, 0));
    cv::putText(img, text, cv::Point(px, py), CV_FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(r, g, b));
}

///
///
///
void Trackball::drawCanvas()
{
    if (_do_display) {
        _canvas.setTo(Scalar::all(0));

        /// Draw source image.
        double radPerPix = _sphere_fov * 1.5 / (2 * DRAW_CELL_DIM);
        static CameraModelPtr draw_camera = CameraModel::createFisheye(
            2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, radPerPix, 360 * CM_D2R);
        static CameraRemapPtr draw_remapper = CameraRemapPtr(new CameraRemap(
            _src_model, draw_camera, _cam_to_roi));
        
        Mat draw_input = _canvas(Rect(0, 0, 2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM));
        draw_remapper->apply(_src_frame, draw_input);

        /// Sphere warping.
        static Mat mapX(_roi_h, _roi_w, CV_32FC1);
        mapX.setTo(Scalar::all(-1));
        static Mat mapY(_roi_h, _roi_w, CV_32FC1);
        mapY.setTo(Scalar::all(-1));
        makeSphereRotMaps(_roi_model, mapX, mapY, _roi_mask, _r_d_ratio, _dr_roi);

        BasicRemapper warper(_roi_w, _roi_h, mapX, mapY);
        static Mat prev_roi = _roi_frame;   // no copy!
        static Mat warp_roi(_roi_h, _roi_w, CV_8UC1);
        warp_roi.setTo(Scalar::all(0));
        warper.apply(prev_roi, warp_roi);
        prev_roi = _roi_frame;  // no copy!

        /// Diff image.
        static Mat diff_roi(_roi_h, _roi_w, CV_8UC1);
        cv::absdiff(_roi_frame, warp_roi, diff_roi);

        /// Draw thresholded ROI.
        static Mat resize_roi(DRAW_CELL_DIM, DRAW_CELL_DIM, CV_8UC1);
        cv::resize(_roi_frame, resize_roi, resize_roi.size());
        Mat draw_roi = _canvas(Rect(2 * DRAW_CELL_DIM, 0, DRAW_CELL_DIM, DRAW_CELL_DIM));
        cv::cvtColor(resize_roi, draw_roi, CV_GRAY2BGR);

        /// Draw warped diff ROI.
        static Mat resize_diff(DRAW_CELL_DIM, DRAW_CELL_DIM, CV_8UC1);
        cv::resize(diff_roi, resize_diff, resize_diff.size());
        Mat draw_diff = _canvas(Rect(3 * DRAW_CELL_DIM, 0, DRAW_CELL_DIM, DRAW_CELL_DIM));
        cv::cvtColor(resize_diff, draw_diff, CV_GRAY2BGR);

        /// Draw current sphere view.
        static Mat resize_view(DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, CV_8UC1);
        cv::resize(_view, resize_view, resize_view.size());
        Mat draw_view = _canvas(Rect(2 * DRAW_CELL_DIM, 1 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, DRAW_CELL_DIM));
        cv::cvtColor(resize_view, draw_view, CV_GRAY2BGR);

        /// Draw current sphere map.
        static Mat resize_map(DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, CV_8UC1);
        cv::resize(_sphere, resize_map, resize_map.size());
        Mat draw_map = _canvas(Rect(2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, DRAW_CELL_DIM));
        cv::cvtColor(resize_map, draw_map, CV_GRAY2BGR);

        /// Draw fictive path.
        //FIXME: add heading arrow to fictive path
        {
            int npts = std::min(int(_pos_heading_hist.size()), FICTIVE_DRAW_POINTS);
            double res = _pos_heading_hist.size() / static_cast<double>(npts);
            double minx = 0, maxx = 0, miny = 0, maxy = 0;
            for (int i = 0; i < npts; i++) {
                int it = static_cast<int>(i * res + 0.5);
                double x = _pos_heading_hist[it].x, y = _pos_heading_hist[it].y;
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

            Mat draw_path = _canvas(Rect(0, 2 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM, DRAW_CELL_DIM));
            double cx = DRAW_CELL_DIM, cy = 0.5 * DRAW_CELL_DIM;
            double ppx = cx, ppy = cy;
            for (int i = 0; i < npts; i++) {
                int it = static_cast<int>(i * res + 0.5);
                double px = cx + scl * _pos_heading_hist[it].y, py = cy - scl * _pos_heading_hist[it].x;
                cv::line(draw_path,
                    cv::Point(static_cast<int>(round(ppx * 16)), static_cast<int>(round(ppy * 16))),
                    cv::Point(static_cast<int>(round(px * 16)), static_cast<int>(round(py * 16))),
                    CV_RGB(255, 255, 255), 1, CV_AA, 4);
                ppx = px;
                ppy = py;
            }
        }

        /// Draw sphere orientation history (animal position history on sphere).
        {
            static const CmPoint up(0, 0, -1.0);
            CmPoint up_roi = up.getTransformed(_roi_to_cam_R.t() * _cam_to_lab_R.t()) * _r_d_ratio;

            double ppx = draw_input.cols / 2, ppy = draw_input.rows / 2;
            // if up vector on front of sphere, change from sphere to cam coordinate frame
            if (up_roi[2] < 0) {
                up_roi[2] += 1; // 1 because we scaled up_roi by _r_d_ratio
                draw_camera->vectorToPixelIndex(up_roi, ppx, ppy);
            }
            for (int i = _R_roi_hist.size() - 1; i >= 0; i--) {
                CmPoint vec = up_roi.getTransformed(_R_roi.t() * _R_roi_hist[i]).getNormalised() * _r_d_ratio;
                
                // sphere is centred at (0,0,1) cam coords, with r
                double px = draw_input.cols / 2.0, py = draw_input.rows / 2.0;

                // cam-relative pos in cam coords
                if (vec[2] < 0) {
                    vec[2] += 1;
                    draw_camera->vectorToPixelIndex(vec, px, py);
                    float d = sqrt((px - ppx) * (px - ppx) + (py - ppy) * (py - ppy));
                    if (d < DRAW_CELL_DIM / 4.0) {
                        float mix = (i + 0.5f) / static_cast<float>(POS_HIST_LENGTH);
                        uint8_t* pdraw_input = &draw_input.at<uint8_t>(static_cast<int>(py), static_cast<int>(px));     // px/py are pixel index values (+0.5)
                        int r = static_cast<int>(mix * pdraw_input[2] + (1 - mix) * 0 + 0.5f);
                        int g = static_cast<int>(mix * pdraw_input[1] + (1 - mix) * 255 + 0.5f);
                        int b = static_cast<int>(mix * pdraw_input[0] + (1 - mix) * 255 + 0.5f);

                        cv::line(draw_input,
                            cv::Point(static_cast<int>(round(px * 16)), static_cast<int>(round(py * 16))),
                            cv::Point(static_cast<int>(round(ppx * 16)), static_cast<int>(round(ppy * 16))),
                            CV_RGB(r, g, b), 1, CV_AA, 4);
                    }
                }
                ppx = px;
                ppy = py;
            }
        }

        /// Draw lines.
        cv::line(_canvas,
            cv::Point(2 * DRAW_CELL_DIM, 0 * DRAW_CELL_DIM) * 16,
            cv::Point(2 * DRAW_CELL_DIM, 3 * DRAW_CELL_DIM) * 16,
            CV_RGB(255, 255, 255), 2, CV_AA, 4);
        cv::line(_canvas,
            cv::Point(0 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM) * 16,
            cv::Point(4 * DRAW_CELL_DIM, 2 * DRAW_CELL_DIM) * 16,
            CV_RGB(255, 255, 255), 2, CV_AA, 4);
        cv::line(_canvas,
            cv::Point(3 * DRAW_CELL_DIM, 0 * DRAW_CELL_DIM) * 16,
            cv::Point(3 * DRAW_CELL_DIM, 1 * DRAW_CELL_DIM) * 16,
            CV_RGB(255, 255, 255), 2, CV_AA, 4);
        cv::line(_canvas,
            cv::Point(2 * DRAW_CELL_DIM, 1 * DRAW_CELL_DIM) * 16,
            cv::Point(4 * DRAW_CELL_DIM, 1 * DRAW_CELL_DIM) * 16,
            CV_RGB(255, 255, 255), 2, CV_AA, 4);

        /// Draw text (with shadow).
        shadowText(_canvas, string("Processed ") + dateString(),
            2, 15,
            0, 255, 255);
        shadowText(_canvas, string("FicTrac (") + string(__DATE__) + string(")"),
            _canvas.cols - 184, 15,
            0, 255, 255);
        shadowText(_canvas, "input image",
            2, 2 * DRAW_CELL_DIM - 8, 
            0, 255, 255);
        shadowText(_canvas, "flat path",
            2, 3 * DRAW_CELL_DIM - 8,
            0, 255, 255);
        shadowText(_canvas, "accumulated map",
            2 * DRAW_CELL_DIM + 3, 3 * DRAW_CELL_DIM - 8,
            0, 255, 255);
        shadowText(_canvas, "sphere ROI",
            2 * DRAW_CELL_DIM + 3, 1 * DRAW_CELL_DIM - 8,
            0, 255, 255);
        shadowText(_canvas, "instant map",
            2 * DRAW_CELL_DIM + 3, 2 * DRAW_CELL_DIM - 8,
            0, 255, 255);
        shadowText(_canvas, "warped diff",
            3 * DRAW_CELL_DIM + 3, 1 * DRAW_CELL_DIM - 8,
            0, 255, 255);
    }
}
