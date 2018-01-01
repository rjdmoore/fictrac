/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SquareRT.h
/// \brief      Minimise perspective R+T transformation of a square.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "typesvars.h"
#include "NLoptFunc.h"

#include <opencv2/opencv.hpp>

#include <vector>

///
///
///
class SquareRT : public NLoptFunc
{
public:
	SquareRT(const std::vector<CmPoint>& cnrs, const cv::Mat& ref_cnrs);

	virtual double objective(unsigned n, const double* x, double* grad);

private:
	const std::vector<CmPoint>& _corners;
    const cv::Mat& _ref_corners;
};
