/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Localiser.cpp
/// \brief      Localises current sphere ROI within surface map.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "Localiser.h"

#include "Logger.h"

using cv::Mat;


///
///
///
Localiser::Localiser(nlopt_algorithm alg, double bound, double tol, int max_evals,
    CameraModelPtr sphere_model, const Mat& sphere_map,
    const Mat& roi_mask, std::shared_ptr<double[]> p1s_lut)
    : _bound(bound), _sphere_model(sphere_model), _sphere_map(sphere_map), _p1s_lut(p1s_lut), _roi_mask(roi_mask)
{
    init(alg, 3);
    setXtol(tol);
    setMaxEval(max_evals);

    if (alg == NLOPT_GN_CRS2_LM) {
        setPopulation(1e3);
    }

    _roi_w = _roi_mask.cols;
    _roi_h = _roi_mask.rows;
}

///
///
///
double Localiser::search(Mat& roi_frame, Mat& R_roi, CmPoint64f& vx)
{
    /// Save current state.
    _roi_frame = roi_frame;
    _R_roi = reinterpret_cast<double*>(R_roi.data);
    double x[3] = { vx[0], vx[1], vx[2] };

    /// Constrain search to bound around guess.
    double lb[3] = { x[0] - _bound, x[1] - _bound, x[2] - _bound };
    double ub[3] = { x[0] + _bound, x[1] + _bound, x[2] + _bound };
    setLowerBounds(lb);
    setUpperBounds(ub);

    /// Run optimisation.
    optimize(x);
    getOptX(x);
    vx.copy(x);
    return getOptF();
}

///
///
///
double Localiser::testRotation(const double x[3])
{
    static double lmat[9];
    CmPoint64f tmp(x[0], x[1], x[2]);
    tmp.omegaToMatrix(lmat);            // relative rotation in camera frame
    const double* rmat = _R_roi;        // pre-multiply to orientation matrix
    static double m[9];                 // absolute orientation in camera frame

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
        const uint8_t* pmask = _roi_mask.ptr(i);
        const uint8_t* proi = _roi_frame.ptr(i);
        const double* v = &(_p1s_lut[i * _roi_w * 3]);
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
            //if (!_sphere_model->vectorToPixelIndex(p2s, px, py)) { continue; }
            _sphere_model->vectorToPixelIndex(p2s, px, py);  // sphere model is spherical, so pixel should never fall outside valid area

            int r = proi[j];
            int s = _sphere_map.data[py * _sphere_map.step + px];
            if (s == 128) { continue; }
            err += (r - s) * (r - s);
            good++;     // number of test pixels that correspond to previously seen pixels
        }
    }

    //LOG_DBG("%d: Tested %.3f %.3f %.3f   total err = %.3e  valid pixels = %d/%d", getNumEval(), x[0], x[1], x[2], err, good, cnt);

    /// Compute avg squared diff error.
    if ((cnt > 0) && (good > (0.25 * static_cast<double>(cnt)))) {
        err /= good;
    } else {
        err = DBL_MAX;
    }
    return err;
}
