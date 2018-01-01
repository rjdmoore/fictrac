/// FicTrac http://rjdmoore.net/fictrac/
/// \file       NLoptFunc.h
/// \brief      Wrapper class for NLopt.
/// \author     Saul Thurrowgood, Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include <nlopt.h>

#include <vector>


///
/// C++ wrapper of the NLopt library for nonlinear optimisation, v2.2+.
/// See:  http://ab-initio.mit.edu/nlopt
///
class NLoptFunc
{
public:
	NLoptFunc();
	virtual ~NLoptFunc();

	///
	/// Run the optimiser.  Default behaviour is to start the search at the
	/// position of the previous result.
	///
	virtual nlopt_result optimize(const double *xInit=0);

	///
	/// Sets the initial value used for the next call to optimize(),
	/// which assumes that call passes NULL as the xInit argument.
	///
	void setXInit(const double *xInit);

	///
	/// Get the optimised value.
	///
	void getOptX(float *x);
	void getOptX(double *x);
	double getOptF();

	///
	/// Returns the number of objective() evaluations by the last optimisation.
	///
	unsigned getNumEval();

	///
	/// Initialise the optimiser.  If 'minimise' is false then the objective
	/// function will be maximised.
	///
	void init(nlopt_algorithm algo, unsigned n, bool minimise=true);

	///
	/// Set initial step size.
	///
	void setInitialStep(const double *dx);

	///
	/// Set stopping criteria.
	///
	void setLowerBounds(const double *lb);
	void setUpperBounds(const double *ub);
	void setLowerBounds(double lb); /// convenience: sets all to same value
	void setUpperBounds(double ub);
	void setFtol(double tol);
	void setXtol(double tol);
	void setMaxEval(unsigned n);

	void getLowerBounds(double *lb);
	void getUpperBounds(double *ub);

	unsigned getDimension();


protected:
	unsigned _nEval;

	///
	/// Objective function executed during optimisation.
	///
	virtual double objective(unsigned n, const double *x, double *grad) = 0;


private:
	static double _cb(unsigned n, const double *x, double *grad, void *data);

	nlopt_opt _opt;
	std::vector<double> _x;
	double _optF;
};
