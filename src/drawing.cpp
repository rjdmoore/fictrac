/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Drawing.cpp
/// \brief      Drawing and other image manipulation methods.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "drawing.h"

#include "geometry.h"

using cv::Mat;
using cv::Point2d;

///
/// Draw a circle using linear circumference segements.
///
void drawCircle_camModel(Mat& img, const CameraModelPtr cam_model, const CmPoint c, const double r, const cv::Scalar colour, bool solid)
{
    const int nsegs = 64;
    const double angleStep = 2 * M_PI / nsegs;
    
    /// Get initial circumference vector.
    CmPoint p0 = c.getRotatedAboutOrthVec(r);
    
    /// Get pixel coords of first point.
    Point2d p1, p2;
    double vec[3] = {p0.x, p0.y, p0.z};
    cam_model->vectorToPixel(vec, p1.x, p1.y);
    
    /// Draw lines between circumference points.
    for (int i = 1; i <= nsegs; i++) {
        /// Rotate initial circumference vector by incrementing angle.
        CmPoint p = p0.getRotatedAboutNorm(c, i * angleStep);
        
        /// Get pixel coords.
        p.copyTo(vec);
        if (!cam_model->vectorToPixel(vec, p2.x, p2.y)) { continue; }
        
        /// Draw dashed/solid.
        if (solid || (i%2)) { cv::line(img, 4*p1, 4*p2, colour, 2, CV_AA, 2); }
        p1 = p2;
    }
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
