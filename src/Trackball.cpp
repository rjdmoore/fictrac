/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Trackball.cpp
/// \brief      Stores surface map and current orientation of tracking ball.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//FIXME: better alg for preferancing high-overlap good matches?

#include "Trackball.h"

#include "geometry.h"
#include "Logger.h"

#include <sstream>
#include <cmath>

using cv::Mat;
using cv::Scalar;

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
Trackball::Trackball(
    int             sphere_map_w,
    int             sphere_map_h,
    CameraModelPtr  roi_model,
    CmPoint64f&     roi_to_cam_r,
    CmPoint64f&     cam_to_lab_r,
    double          sphere_diam_rad,
    double          error_thresh,
    int             max_bad_frames = 1,
    Mat&            mask = Mat(),
    Mat&            sphere_template = Mat(),
    bool            do_display = false,
    double          bound = 0.5,
    double          tol = 1e-4,
    int             max_eval = 100
) : _map_w(sphere_map_w), _map_h(sphere_map_h),
    _roi_w(roi_model->width()), _roi_h(roi_model->height()),
    _roi_model(roi_model),
    _error_thresh(error_thresh), _bound(bound), _err(0),
    _max_bad_frames (max_bad_frames), _reset(true),
    _cnt(0), _seq(0),
    _intx(0), _inty(0), _posx(0), _posy(0), _heading(0),
    _do_display(do_display)
{
    // NOTE: sin rather than tan to correct for apparent size of sphere
    _r_d_ratio = sin(sphere_diam_rad / 2.0);

    /*
    * The model sphere has it's own coordinate system, because we arbitrarily set
    * the view vector corresponding to the centre of the projection of the tracking
    * ball to be the principal axis of the virtual camera (cam_model).
    *
    * All incoming and outgoing vectors and matrices must thus be transposed to/from
    * this coordinate system: lab <= (_cam_to_lab_R) => cam <= (_roi_to_cam_R) => roi.
    */

    _roi_to_cam_R = CmPoint64f::omegaToMatrix(roi_to_cam_r);
    _cam_to_lab_R = CmPoint64f::omegaToMatrix(cam_to_lab_r);

    /// Init minimiser.
    init(NLOPT_LN_BOBYQA, 3);
    setLowerBounds(-_bound);
    setUpperBounds(_bound);
    setFtol(tol);
    setXtol(tol);
    setMaxEval(max_eval);

    /// Surface mapping.
    _sphere_model = CameraModel::createEquiArea(_map_w, _map_h);

    /// Init orientation.
    _R_roi = Mat::eye(3, 3, CV_64F);
    // _dr_roi is init to (0,0,0) by default

    /// Mask.
    if (!mask.empty()) {
        _mask = mask.clone();
    } else {
        _mask.create(_roi_h, _roi_w, CV_8U);
        _mask.setTo(Scalar::all(255));
    }

    /// Buffers.
    _sphere.create(_map_h, _map_w, CV_8U);
    _sphere.setTo(Scalar::all(128));
    _sphere_max.create(_map_h, _map_w, CV_32S);
    _sphere_max.setTo(Scalar::all(0));
    _sphere_hist.create(_map_h, _map_w * 3, CV_32S);	// hist stores histogram for 3 classes - black, grey, white
    _sphere_hist.setTo(Scalar::all(0));

    /// Surface map template.
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
        uint8_t* pmask = _mask.ptr(i);
        for (int j = 0; j < _roi_w; j++) {
            if (pmask[j] < 255) { continue; }

            double l[3] = { 0 };
            _roi_model->pixelIndexToVector(j, i, l);
            vec3normalise(l);

            double* s = &_p1s_lut[(i * _roi_w + j) * 3];
            if (!intersectSphere(l, s, _r_d_ratio)) { pmask[j] = 128; }
        }
    }

    /// Data recording.
    _log = std::unique_ptr<Recorder>(new Recorder("fictrac.dat"));

    /// Display.
    if (_do_display) {
        _orient.create(_map_h, _map_w, CV_8U);
        _orient.setTo(Scalar::all(255));
    }
}

///
/// Default destructor.
///
Trackball::~Trackball()
{}

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
}

///
///
///
bool Trackball::update(cv::Mat view, double timestamp, bool allow_global)
{
    bool ret = true;
    static int nbad = 0;

    /// Localise current view on sphere.
    setCurrentView(view, timestamp);
    if (!doSearch(allow_global)) {
        LOG_ERR("Error! Could not match current sphere orientation to within error threshold (%d). No data will be output for this frame!", _error_thresh);
        nbad++;
    }
    else {
        updateSphere();
        updatePath();
        logData();
    }

    /// Handle failed localisation.
    if ((_max_bad_frames >= 0) && (nbad > _max_bad_frames)) {
        _seq = 0;
        nbad = 0;
        reset();
        ret = false;
    }
    else {
        _cnt++;
        _seq++;
    }

    /// Clear reset flag.
    _reset = false;
    
    return ret;
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
        _orient.setTo(Scalar::all(128));
    }

    Mat p2s(3, 1, CV_64F);
    int cnt = 0, good = 0;
    for (int i = 0; i < _roi_h; i++) {
        uint8_t* pview = _roi.ptr(i);
        for (int j = 0; j < _roi_w; j++) {
            if (_mask.at<uint8_t>(i, j) < 255) { continue; }
            cnt++;

            // rotate point about rotation axis (sphere coords)
            Mat p1s(3, 1, CV_64F, &_p1s_lut[(i * _roi_w + j) * 3]);
            p2s = _R_roi * p1s;

            // map vector in sphere coords to pixel
            int x = 0, y = 0;
            if (!_sphere_model->vectorToPixelIndex(reinterpret_cast<double*>(p2s.data), x, y)) { continue; }

            int32_t* smax = &_sphere_max.at<int32_t>(y, x);
            if (*smax > 0) { good++; }

            int h = ++(_sphere_hist.at<int32_t>(y, x * 3 + static_cast<int>(pview[j]/127.f)));	// thresholded sphere is quantised to levels 0 = black, 1 = grey, 2 = white
            if (h > *smax) {
                *smax = h;
                _sphere.at<uint8_t>(y, x) = pview[j];
            }

            // display
            if (_do_display) { _orient.at<uint8_t>(y, x) = pview[j]; }
        }
    }

    LOG_DBG("Match overlap: %.1f", 100 * good / static_cast<double>(cnt));
}

///
///
///
void Trackball::updatePath()
{
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

    // update data frame count and sequence number
    _cnt++;
    _seq++;
}

///
///
///
bool Trackball::logData()
{
    // frame_count
    std::stringstream ss;
    ss.precision(14);
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
    // timestamp
    ss << _ts << ", " << _seq << ", ";
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
            if (_mask.at<uint8_t>(i, j) < 255) { continue; }
            cnt++;

            // rotate point about rotation axis (sphere coords)
            Mat p1s(3, 1, CV_64F, &_p1s_lut[(i * _roi_w + j) * 3]);
            p2s = testR * p1s;

            // map vector in sphere coords to pixel
            int x = 0, y = 0;
            if (!_sphere_model->vectorToPixelIndex(reinterpret_cast<double*>(p2s.data), x, y)) { continue; }

            if (_sphere_max.at<int32_t>(y, x) == 0) { continue; }
            int px = _roi.at<uint8_t>(i, j);
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
void Trackball::drawDebug(Mat& sphere, Mat& orient)
{
    if (_do_display) {
        cv::resize(_sphere, sphere, sphere.size());
        cv::resize(_orient, orient, orient.size());
    }
}
