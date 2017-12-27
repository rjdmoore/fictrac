/// FicTrac http://rjdmoore.net/fictrac/
/// \file       Drawing.h
/// \brief      Drawing and other image manipulation methods.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "typesvars.h"
#include "CameraModel.h"

#include <opencv2/opencv.hpp>

/// Draw a circle using linear circumference segements.
void drawCircle_camModel(cv::Mat& img, const CameraModelPtr cam_model, const CmPoint c, const double r, const cv::Scalar colour, bool solid=true);

/// Stretch contrast of grey image to cover range [0,255].
void histStretch(cv::Mat& grey);

/// Draw cursor cross.
void drawCursor(cv::Mat& rgb, const cv::Point2d& pt, cv::Scalar colour);
