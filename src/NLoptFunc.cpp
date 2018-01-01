/// FicTrac http://rjdmoore.net/fictrac/
/// \file       NLoptFunc.cpp
/// \brief      Wrapper class for NLopt.
/// \author     Saul Thurrowgood, Richard Moore
/// \copyright  CC BY-NC-SA 3.0

//TODO: change printf to logger msgs.

#include "NLoptFunc.h"

#include <iostream>
#include <cstdio>
#include <stdlib.h>

template <typename T>
static inline T clamp(T x, T min, T max)
{
	return x < min ? min : x > max ? max : x;
}

NLoptFunc::NLoptFunc()
{
}

NLoptFunc::~NLoptFunc()
{
	nlopt_destroy(_opt);
	_opt = 0;
}

void NLoptFunc::init(nlopt_algorithm algo, unsigned n, bool minimise)
{
	_opt = nlopt_create(algo, n);
	if (!_opt) {
		fprintf(stderr, "NLoptFunc: nlopt_create failed\n");
		abort();
	}

	if (minimise) {
		nlopt_set_min_objective(_opt, _cb, this);
	} else {
		nlopt_set_max_objective(_opt, _cb, this);
	}

	_nEval = 0;
	_x.clear();
	_x.resize(n, 0.0);
	_optF = 0;
}

void NLoptFunc::setXInit(const double *xInit)
{
	if (xInit) {
		for (unsigned i=0; i<_x.size(); ++i)
			_x[i] = xInit[i];
	}
}

nlopt_result NLoptFunc::optimize(const double *xInit)
{
	if (!_opt) {
		fprintf(stderr, "NLoptFunc: not initialised\n");
		abort();
	}

	if (xInit) {
		for (unsigned i=0; i<_x.size(); ++i)
			_x[i] = xInit[i];
	} else {
		/// Default is to start with the previous result, already in _x.
	}

	/// Clamp initial _x value to within bounds by a certain fraction of range.
	const double boundaryFraction = 0.01;
	double lb[_x.size()];
	double ub[_x.size()];
	nlopt_get_lower_bounds(_opt, lb);
	nlopt_get_upper_bounds(_opt, ub);
	for (unsigned i=0; i<_x.size(); ++i) {
		double a = lb[i];
		double b = ub[i];
		if (a > b)
			std::swap(a,b);
		double offset = (b - a) * boundaryFraction;
		_x[i] = clamp(_x[i], a+offset, b-offset);
	}

	_nEval = 0;
	_optF = 0;
	return nlopt_optimize(_opt, &_x[0], &_optF);
}

///
/// Static intermediate callback function that calls actual objective through
/// the given data pointer.
///
double NLoptFunc::_cb(unsigned n, const double *x, double *grad, void *data)
{
	NLoptFunc& func = *reinterpret_cast<NLoptFunc*>(data);
	++func._nEval;
	return func.objective(n, x, grad);
}

double NLoptFunc::objective(unsigned n,const double *x,double *grad)
	{return 0;}

void NLoptFunc::setLowerBounds(const double *lb)
	{ nlopt_set_lower_bounds(_opt, lb); }

void NLoptFunc::setUpperBounds(const double *ub)
	{ nlopt_set_upper_bounds(_opt, ub); }

void NLoptFunc::getLowerBounds(double *lb)
	{ nlopt_get_lower_bounds(_opt, lb); }

void NLoptFunc::getUpperBounds(double *ub)
	{ nlopt_get_upper_bounds(_opt, ub); }

void NLoptFunc::setLowerBounds(double lb)
	{ nlopt_set_lower_bounds1(_opt, lb); }

void NLoptFunc::setUpperBounds(double ub)
	{ nlopt_set_upper_bounds1(_opt, ub); }

void NLoptFunc::setFtol(double tol)
	{ nlopt_set_ftol_rel(_opt, tol); }

void NLoptFunc::setXtol(double tol)
	{ nlopt_set_xtol_rel(_opt, tol); }

void NLoptFunc::setMaxEval(unsigned n)
	{ nlopt_set_maxeval(_opt, n); }

unsigned NLoptFunc::getNumEval()
	{ return _nEval; }

void NLoptFunc::getOptX(float *x)
	{ for (unsigned i=0; x && i<_x.size(); ++i) x[i] = _x[i]; }

void NLoptFunc::getOptX(double *x)
	{ for (unsigned i=0; x && i<_x.size(); ++i) x[i] = _x[i]; }

double NLoptFunc::getOptF()
	{ return _optF; }

void NLoptFunc::setInitialStep(const double *dx)
	{ nlopt_set_initial_step(_opt, dx); }

unsigned NLoptFunc::getDimension()
	{ return nlopt_get_dimension(_opt); }
