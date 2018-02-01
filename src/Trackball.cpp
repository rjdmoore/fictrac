/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Trackball.cpp
/// \brief      Stores surface map and current orientation of tracking ball.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "Trackball.h"

#include "geometry.h"
#include "logging.h"

using cv::Mat;
using cv::Scalar;

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
    CameraModelPtr  cam_model,
    int             sphere_w,
    int             sphere_h,
    double          sphere_r_d_ratio,
    cv::Mat&        mask = cv::Mat(),
    cv::Mat&        sphere_template = cv::Mat(),
    bool            do_display = false,
    bool            do_update = true,
    double          lower_bound = -0.5,
    double          upper_bound = 0.5,
    double          tol = 1e-4,
    int             max_eval = 100
) : _cam_model(cam_model), _w(sphere_w), _h(sphere_h),
    _view_w(cam_model->width()), _view_h(cam_model->height()),
    _r(sphere_r_d_ratio), _do_display(do_display)
{
    /*
    * The model sphere has it's own coordinate system, because we arbitrarily set
    * the view vector corresponding to the centre of the projection of the tracking
    * ball to be the principal axis of the virtual camera (cam_model).
    *
    * All incoming and outgoing vectors and matrices must thus be transposed to/from
    * this coordinate system: world <= (Rw) => cam <= (roi_rot_mat) => ROI.
    */

    /// Init minimiser.
    init(NLOPT_LN_BOBYQA, 3);
    setLowerBounds(lower_bound);
    setUpperBounds(upper_bound);
    setFtol(tol);
    setXtol(tol);
    setMaxEval(max_eval);

    /// Surface mapping.
    _sphere_model = CameraModel::createEquiArea(_w, _h);

    /// Init orientation.
    _R = Mat::eye(3, 3, CV_64F);

    /// Mask.
    if (!mask.empty()) {
        _mask = mask.clone();
    } else {
        _mask.create(_view_h, _view_w, CV_8U);
        _mask.setTo(Scalar::all(255));
    }

    /// Buffers.
    _sphere.create(_h, _w, CV_8U);
    _sphere_max.create(_h, _w, CV_32S);
    _sphere_hist.create(_h, _w * 3, CV_32S);
    clearSphere();

    /// Surface map template.
    if (!sphere_template.empty()) {
        assert(sphere_template.size() == _sphere.size());
        sphere_template.copyTo(_sphere);
        for (int i = 0; i < _h; i++) {
            int32_t* pmax = (int32_t*)_sphere_max.ptr(i);
            int32_t* phist = (int32_t*)_sphere_hist.ptr(i);
            uint8_t* psphere = _sphere.ptr(i);
            for (int j = 0; j < _w; j++) {
                pmax[j] = 10;   // arbitrary start weight
                phist[j * 3 + static_cast<int>(psphere[j]/127.f)] = pmax[j];
            }
        }
    }

    /// Display.
    _orient.create(_h, _w, CV_8U);
    _orient.setTo(Scalar::all(255));

    /// Pre-calc view rays.
    _p1s_lut = boost::shared_array<double>(new double[_view_w * _view_h * 3]);
    memset(_p1s_lut.get(), 0, _view_w * _view_h * 3 * sizeof(double));
    for (int i = 0; i < _view_h; i++) {
        uint8_t* pmask = _mask.ptr(i);
        for (int j = 0; j < _view_w; j++) {
            if (pmask[j] < 255) { continue; }

            double l[3] = { 0 };
            _cam_model->pixelIndexToVector(j, i, l);
            vec3normalise(l);

            double* s = &_p1s_lut[(i * _view_w + j) * 3];
            if (!intersectSphere(l, s, _r)) { pmask[j] = 128; }
        }
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
void Trackball::clearSphere()
{
    _sphere.setTo(Scalar::all(128));
    _sphere_max.setTo(Scalar::all(0));
    _sphere_hist.setTo(Scalar::all(0));
}

///
///
///
cv::Mat& Trackball::updateSphere(CmPoint64f& angleAxis)
{
    Mat tmpR = CmPoint64f::omegaToMatrix(angleAxis);    // relative rotation in camera frame
    _R = _R * tmpR;                                     // absolute orientation in camera frame (save)

    if (_do_display) {
        _orient.setTo(Scalar::all(128));
    }

    Mat p2s(3, 1, CV_64F);
    int cnt = 0, good = 0;
    for (int i = 0; i < _view_h; i++) {
        uint8_t* pview = _view.ptr(i);
        for (int j = 0; j < _view_w; j++) {
            if (_mask.at<uint8_t>(i, j) < 255) { continue; }
            cnt++;

            // rotate point about rotation axis (sphere coords)
            Mat p1s(3, 1, CV_64F, &_p1s_lut[(i * _view_w + j) * 3]);
            p2s = _R * p1s;

            // map vector in sphere coords to pixel
            int x = 0, y = 0;
            if (!_sphere_model->vectorToPixelIndex(reinterpret_cast<double*>(p2s.data), x, y)) { continue; }

            int32_t* smax = &_sphere_max.at<int32_t>(y, x);
            if (*smax > 0) { good++; }

            int h = ++(_sphere_hist.at<int32_t>(y, x * 3 + static_cast<int>(pview[j]/127.f)));
            if (h > *smax) {
                *smax = h;
                _sphere.at<uint8_t>(y, x) = pview[j];
            }

            // display
            if (_do_display) { _orient.at<uint8_t>(y, x) = pview[j]; }
        }
    }

    BOOST_LOG_TRIVIAL(debug) << "Match overlap: " << 100 * good / static_cast<double>(cnt);

    return _R;
}

///
///
///
double Trackball::testRotation(const double x[3])
{
    Mat tmpR = CmPoint64f::omegaToMatrix(CmPoint64f(x[0], x[1], x[2])); // relative rotation in camera frame
    Mat testR = _R * tmpR;                                              // absolute orientation in camera frame

    double err = 0;
    Mat p2s(3, 1, CV_64F);
    int cnt = 0, good = 0;
    for (int i = 0; i < _view_h; i++) {
        for (int j = 0; j < _view_w; j++) {
            if (_mask.at<uint8_t>(i, j) < 255) { continue; }
            cnt++;

            // rotate point about rotation axis (sphere coords)
            Mat p1s(3, 1, CV_64F, &_p1s_lut[(i * _view_w + j) * 3]);
            p2s = testR * p1s;

            // map vector in sphere coords to pixel
            int x = 0, y = 0;
            if (!_sphere_model->vectorToPixelIndex(reinterpret_cast<double*>(p2s.data), x, y)) { continue; }

            if (_sphere_max.at<int32_t>(y, x) == 0) { continue; }
            int px = _view.at<uint8_t>(i, j);
            if (px == 128) { continue; }
            double diff = (px - static_cast<int>(_sphere.at<uint8_t>(y, x)));
            err += diff*diff;
            good++;
        }
    }

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
