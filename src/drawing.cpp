/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Drawing.cpp
/// \brief      Drawing and other image manipulation methods.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "drawing.h"

#include "geometry.h"

/// OpenCV individual includes required by gcc?
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

using std::shared_ptr;
using std::vector;
using cv::Mat;
using cv::Point2d;
using cv::Point2i;

///
/// Draw a circle using linear circumference segements.
///
shared_ptr<vector<Point2d>> projCircle(const CameraModelPtr cam_model, const CmPoint c, const double r)
{
    const int nsegs = 64;
    const double angleStep = 2 * CM_PI / nsegs;

    /// Get initial circumference vector.
    CmPoint p0 = c.getRotatedAboutOrthVec(r);

    /// Get pixel coords of first point.
    Point2d p1;
    double vec[3] = { p0.x, p0.y, p0.z };
    cam_model->vectorToPixel(vec, p1.x, p1.y);

    shared_ptr<vector<Point2d>> circ_pts = shared_ptr<vector<Point2d>>(new vector<Point2d>());
    circ_pts->push_back(p1);

    /// Interpolate circumference.
    for (int i = 1; i <= nsegs; i++) {
        /// Rotate initial circumference vector by incrementing angle.
        CmPoint p = p0.getRotatedAboutNorm(c, i * angleStep);

        /// Get pixel coords.
        p.copyTo(vec);
        if (!cam_model->vectorToPixel(vec, p1.x, p1.y)) { continue; }

        circ_pts->push_back(p1);
    }

    return circ_pts;
}

shared_ptr<vector<Point2i>> projCircleInt(const CameraModelPtr cam_model, const CmPoint c, const double r)
{
    shared_ptr<vector<Point2d>> circ_pts = projCircle(cam_model, c, r);
    shared_ptr<vector<Point2i>> circ_pts_int = shared_ptr<vector<Point2i>>(new vector<Point2i>());
    for (Point2d pd : *circ_pts) { circ_pts_int->push_back(pd); }
    return circ_pts_int;
}

void drawCircle(cv::Mat& img, shared_ptr<vector<cv::Point2d>> circ_pts, const cv::Scalar colour, bool solid)
{
    if (circ_pts->empty()) { return; }

    /// Draw lines between circumference points.
    Point2d p1 = circ_pts->front(), p2;
    for (int i = 1; i < circ_pts->size(); i++) {
        /// Draw dashed/solid.
        p2 = (*circ_pts)[i];
        if (solid || (i % 2)) { cv::line(img, 4 * p1, 4 * p2, colour, 2, CV_AA, 2); }
        p1 = p2;
    }
}

void drawCircle_camModel(Mat& img, const CameraModelPtr cam_model, const CmPoint c, const double r, const cv::Scalar colour, bool solid)
{
    shared_ptr<vector<Point2d>> circ_pts = projCircle(cam_model, c, r);
    drawCircle(img, circ_pts, colour, solid);
}

///
/// Stretch contrast of grey image to cover range [0,255].
/// Produces 1% under- and over-saturated pixels.
///
void histStretch(Mat& grey)
{
	static int hist[256];
	memset(hist, 0, 256*sizeof(int));
    
    /// Construct histogram.
	for( int i = 0; i < grey.rows; i++ ) {
		uint8_t *pix = grey.ptr(i);
		for( int j = 0; j < grey.cols; j++ ) {
            hist[pix[j]]++;
		}
	}
    
    /// Find 1% and 99% percentiles.
    int running_cnt = 0;
    int n = grey.cols * grey.rows;
	double m1 = n*0.01, m2 = n*0.99;
	int m1_i = 0, m2_i = 255;
	for( int i = 0; i < 256; i++ ) {
		running_cnt += hist[i];
        
        if (running_cnt < m1) { m1_i = i; }
        if (running_cnt < m2) { m2_i = i; }
	}

    /// Stretch percentiles to cover range [0,255].
	for( int i = 0; i < grey.rows; i++ ) {
		uint8_t *pix = grey.ptr(i);
		for( int j = 0; j < grey.cols; j++ ) {
            pix[j] = clamp(int(double(pix[j] - m1_i) * 255.0 / double(m2_i - m1_i) + 0.5), 0, 255);
		}
	}
}

///
/// Draw cursor cross.
///
void drawCursor(Mat& rgb, const Point2d& pt, cv::Scalar colour)
{
    const int inner_rad = std::max(int(rgb.cols/500+0.5), 2);
    const int outer_rad = std::max(int(rgb.cols/150+0.5), 5);

    cv::line(rgb, pt-Point2d(outer_rad,outer_rad), pt-Point2d(inner_rad,inner_rad), colour, 1, CV_AA);
    cv::line(rgb, pt+Point2d(inner_rad,inner_rad), pt+Point2d(outer_rad,outer_rad), colour, 1, CV_AA);
    cv::line(rgb, pt-Point2d(-outer_rad,outer_rad), pt-Point2d(-inner_rad,inner_rad), colour, 1, CV_AA);
    cv::line(rgb, pt+Point2d(-inner_rad,inner_rad), pt+Point2d(-outer_rad,outer_rad), colour, 1, CV_AA);
}

///
/// Draw transformed axes.
///
void drawAxes(Mat& rgb, const CameraModelPtr cam_model, const Mat& R, const Mat& t, const cv::Scalar colour)
{    
    /// Transformed axes.
    Mat sx = R * (cv::Mat_<double>(3,1) << 1,0,0) + t;
    Mat sy = R * (cv::Mat_<double>(3,1) << 0,1,0) + t;
    Mat sz = R * (cv::Mat_<double>(3,1) << 0,0,1) + t;
    
    /// Draw transformed axes.
    double vec[3];
    Point2d pt, pt0;
    vec[0] = t.at<double>(0,0);
    vec[1] = t.at<double>(1,0);
    vec[2] = t.at<double>(2,0);
    cam_model->vectorToPixel(vec, pt0.x, pt0.y);
    
    // x
    {
        vec[0] = sx.at<double>(0, 0);
        vec[1] = sx.at<double>(1, 0);
        vec[2] = sx.at<double>(2, 0);
        cam_model->vectorToPixel(vec, pt.x, pt.y);

        cv::line(rgb, 4 * pt0, 4 * pt, colour, 2, CV_AA, 2);
        cv::putText(rgb, "x", pt + Point2d(10, 0), cv::FONT_HERSHEY_SIMPLEX, 1.0, colour, 1, CV_AA);
        // indicate in to or out of the page
        if (vec[2] < t.at<double>(2, 0)) {
            cv::circle(rgb, 4 * pt, 4 * 4, colour, 2, CV_AA, 2);
        }
        else {
            cv::line(rgb, 4 * (pt + Point2d(-4, -4)), 4 * (pt + Point2d(4, 4)), colour, 2, CV_AA, 2);
            cv::line(rgb, 4 * (pt + Point2d(-4, 4)), 4 * (pt + Point2d(4, -4)), colour, 2, CV_AA, 2);
        }
    }
    
    // y
    {
        vec[0] = sy.at<double>(0, 0);
        vec[1] = sy.at<double>(1, 0);
        vec[2] = sy.at<double>(2, 0);
        cam_model->vectorToPixel(vec, pt.x, pt.y);

        cv::line(rgb, 4 * pt0, 4 * pt, colour, 2, CV_AA, 2);
        cv::putText(rgb, "y", pt + Point2d(10, 0), cv::FONT_HERSHEY_SIMPLEX, 1.0, colour, 1, CV_AA);
        // indicate in to or out of the page
        if (vec[2] < t.at<double>(2, 0)) {
            cv::circle(rgb, 4 * pt, 4 * 4, colour, 2, CV_AA, 2);
        }
        else {
            cv::line(rgb, 4 * (pt + Point2d(-4, -4)), 4 * (pt + Point2d(4, 4)), colour, 2, CV_AA, 2);
            cv::line(rgb, 4 * (pt + Point2d(-4, 4)), 4 * (pt + Point2d(4, -4)), colour, 2, CV_AA, 2);
        }
    }
    
    // z
    {
        vec[0] = sz.at<double>(0, 0);
        vec[1] = sz.at<double>(1, 0);
        vec[2] = sz.at<double>(2, 0);
        cam_model->vectorToPixel(vec, pt.x, pt.y);

        cv::line(rgb, 4 * pt0, 4 * pt, colour, 2, CV_AA, 2);
        cv::putText(rgb, "z", pt + Point2d(10, 0), cv::FONT_HERSHEY_SIMPLEX, 1.0, colour, 1, CV_AA);
        // indicate in to or out of the page
        if (vec[2] < t.at<double>(2, 0)) {
            cv::circle(rgb, 4 * pt, 4 * 4, colour, 2, CV_AA, 2);
        }
        else {
            cv::line(rgb, 4 * (pt + Point2d(-4, -4)), 4 * (pt + Point2d(4, 4)), colour, 2, CV_AA, 2);
            cv::line(rgb, 4 * (pt + Point2d(-4, 4)), 4 * (pt + Point2d(4, -4)), colour, 2, CV_AA, 2);
        }
    }
}

///
/// Draw rectangle corners.
///
void drawRectCorners(Mat& rgb, const CameraModelPtr cam_model, Mat& cnrs, const cv::Scalar colour)
{
    assert((cnrs.rows == 3) && (cnrs.cols == 4));
    
    Point2d pt;
    double vec[3];
    for (int i = 0; i < 4; i++) {
        vec[0] = cnrs.at<double>(0,i);
        vec[1] = cnrs.at<double>(1,i);
        vec[2] = cnrs.at<double>(2,i);
        vec3normalise(vec); 
        if (cam_model->vectorToPixel(vec, pt.x, pt.y)) {
            drawCursor(rgb, pt, colour);
        }
    }
}

///
/// Place text with shadow.
///
void shadowText(Mat& img, std::string text, int px, int py, int r, int g, int b)
{
    cv::putText(img, text, cv::Point(px + 1, py + 1), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 0, 0));
    cv::putText(img, text, cv::Point(px, py), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(r, g, b));
}
