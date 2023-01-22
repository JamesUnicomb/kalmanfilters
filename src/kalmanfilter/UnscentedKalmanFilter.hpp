#ifndef _UKF_HPP_
#define _UKF_HPP_

#include "nr3.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"
#include <vector>
#include <cmath>

template <typename F, typename H>
class UnscentedKalmanFilter
{
public:
	/*
	X -> state sigma points
	Z -> measurement sigma points
	W -> weights

    X = [f(x,dt) for x in X]
    xbar = sum(w * x for x in X, w in W)
    sigma = [w * (x - xbar) * (x - xbar)^T for x in X, w in W] + Q

    Z = [h(x) for x in X]
	zbar = sum(w * z for z in Z, w in W)
	S = [w * (z - zbar) * (z - zbar)^T for z in Z, w in W] + R
	P = [w * (x - xbar) * (z - zbar)^T for x in X, z in Z, w in W] 

	K = P * S^-1

    y = z - zbar
    xbar = xbar + K * y
    sigma = sigma - K * S * K^T
    */

	UnscentedKalmanFilter(double q, linalg::Vector state, linalg::Matrix state_unc)
		: state(state)
		, state_unc(state_unc)
		, statedim(h.statedim)
		, measuredim(h.measuredim)
		, svdstate(h.statedim, h.statedim)
		, svdinnovation(h.measuredim, h.measuredim)
	{
		int i;

		sigmadim = 2 * statedim + 1;

		// ukf parameters
		// see https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf
		ap = 0.01;
		bt = 2.0;
		kp = 0.01;
		ld = ap * ap * (statedim + kp) - statedim;

		wm = linalg::Vector(sigmadim, 1.0 / (2.0 * (statedim + ld)));
		wc = linalg::Vector(sigmadim, 1.0 / (2.0 * (statedim + ld)));
		wm[0] = ld / (statedim + ld);
		wc[0] = ld / (statedim + ld) + (1 - ap * ap + bt);

		// set the measurement
		// noise for the motion model
		f.q = q;

		state_unc_sqrtm = linalg::Matrix(statedim, statedim, 0.0);
		state_sigma_points = std::vector<linalg::Vector>(sigmadim, linalg::Vector(statedim, 0.0));

		// expected measurement vector and measurement sigma points
		measurement = linalg::Vector(measuredim, 0.0);
		measurement_sigma_points = std::vector<linalg::Vector>(sigmadim, linalg::Vector(measuredim, 0.0));

		// process uncertainty matrix
		process_unc = linalg::Matrix(statedim, statedim, 0.0);

		// innovation is Nx1
		innovation = linalg::Vector(measuredim, 0.0);
		innovation_unc = linalg::Matrix(measuredim, measuredim, 0.0);
		innovation_unc_inv = linalg::Matrix(measuredim, measuredim, 0.0);

		// measurement and state covariance
		state_measurement_cov = linalg::Matrix(statedim, measuredim, 0.0);

		// kalman gain is NxM
		gain = linalg::Matrix(statedim, measuredim, 0.0);
		gainT = linalg::Matrix(measuredim, statedim, 0.0);

		// update variables
		dx = linalg::Vector(statedim, 0.0);

		// temporary matrices
		tmpstate = linalg::Vector(statedim, 0.0);
		tmpnn = linalg::Matrix(statedim, statedim, 0.0);
		tmpnm = linalg::Matrix(statedim, measuredim, 0.0);
		tmpmn = linalg::Matrix(measuredim, statedim, 0.0);
		tmpmm = linalg::Matrix(measuredim, measuredim, 0.0);
		tmpunc = linalg::Matrix(statedim, statedim, 0.0);
	}

	void set_state(linalg::Vector& state_)
	{
		state = state_;
	}

	void set_state_unc(linalg::Matrix& state_unc_)
	{
		state_unc = state_unc_;
	}

	linalg::Vector get_state()
	{
		return state;
	}

	linalg::Matrix get_state_unc()
	{
		return state_unc;
	}

	linalg::Vector get_innovation()
	{
		return innovation;
	}

	linalg::Matrix get_innovation_unc()
	{
		return innovation_unc;
	}

	void predict(double delta)
	{
		int i;
		f.getProcessUncertainty(delta, state, process_unc);

		// calculate sigma points
		svdstate.dcmp(state_unc);
		svdstate.sqrtm(state_unc_sqrtm);

		state_sigma_points[0] = state;
		for(i = 0; i < statedim; i++)
		{
			tmpstate = linalg::Vector(statedim, state_unc_sqrtm[i]);
			linalg::mult(sqrt(statedim + ld), tmpstate, tmpstate);
			linalg::add(state, tmpstate, state_sigma_points[i + 1]);
			linalg::subtract(state, tmpstate, state_sigma_points[i + statedim + 1]);
		}

		// make prediction for each sigma point
		for(i = 0; i < sigmadim; i++)
		{
			f.predict(delta, state_sigma_points[i]);
		}

		linalg::weightedsum(wm, state_sigma_points, state);
		linalg::weightedmult(wc, state_sigma_points, state, state_sigma_points, state, state_unc);
		linalg::add(state_unc, process_unc, state_unc);

		f.final(state, state_unc);
	}

	template <typename Z>
	void update(Z& z)
	{
		for(int i = 0; i < sigmadim; i++)
		{
			h.predict(state_sigma_points[i], z, measurement_sigma_points[i]);
		}
		linalg::weightedsum(wm, measurement_sigma_points, measurement);
		linalg::weightedmult(
			wc, measurement_sigma_points, measurement, measurement_sigma_points, measurement, innovation_unc);
		innovation_unc += z.unc();

		svdinnovation.dcmp(innovation_unc);
		svdinnovation.inverse(innovation_unc_inv);

		linalg::weightedmult(
			wc, state_sigma_points, state, measurement_sigma_points, measurement, state_measurement_cov);

		// calculate kalman gain
		linalg::mult(state_measurement_cov, innovation_unc_inv, gain);

		// update state
		linalg::subtract(z.vec(), measurement, innovation);
		linalg::mult(gain, innovation, dx);
		state += dx;

		// update covariance
		linalg::transpose(gain, gainT);
		linalg::mult(innovation_unc, gainT, tmpmn);
		linalg::mult(gain, tmpmn, tmpunc);
		linalg::subtract(state_unc, tmpunc, state_unc);

		// update normalization routine
		h.final(state, state_unc);
	}

private:
	F f;
	H h;

	int statedim;
	int measuredim;
	int sigmadim;

	// ukf paramertes (see Probabilistic Robitcs by Thrun, Burgard, Fox)
	double ap;
	double bt;
	double kp;
	double ld;

	// x and sigma
	linalg::Vector state;
	linalg::Matrix state_unc;
	linalg::Matrix state_unc_sqrtm;
	std::vector<linalg::Vector> state_sigma_points;

	// z and Z
	linalg::Vector measurement;
	std::vector<linalg::Vector> measurement_sigma_points;

	// P
	linalg::Matrix state_measurement_cov;

	// Q
	linalg::Matrix process_unc;

	// y and S
	linalg::Vector innovation;
	linalg::Matrix innovation_unc;

	// K
	linalg::Matrix gain;
	linalg::Matrix gainT;

	// ukf weights
	linalg::Vector wm;
	linalg::Vector wc;

	// temp vectors/matrices for calculations
	linalg::Matrix innovation_unc_inv;
	linalg::Vector dx;
	linalg::Matrix eye, tmpunc;
	linalg::Vector tmpstate;
	linalg::Matrix tmpnn, tmpnm, tmpmn, tmpmm;

	// matrix inversion and sigma points
	linalg::SVD svdinnovation;
	linalg::SVD svdstate;
};

#endif /* _UKF_HPP_ */