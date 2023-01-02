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

	// x and sigma
	std::vector<double> state;
	std::vector<std::vector<double>> state_unc;
	std::vector<std::vector<double>> state_unc_sqrtm;
	std::vector<std::vector<double>> state_sigma_points;

	// z and Z
	std::vector<double> measurement;
	std::vector<std::vector<double>> measurement_sigma_points;

	// P
	std::vector<std::vector<double>> state_measurement_cov;

	// Q and R
	std::vector<std::vector<double>> process_unc, measure_unc;

	// y and S
	std::vector<double> innovation;
	std::vector<std::vector<double>> innovation_unc;

	// K
	std::vector<std::vector<double>> gain;
	std::vector<std::vector<double>> gainT;

	// ukf weights
	std::vector<double> wm;
	std::vector<double> wc;

	UnscentedKalmanFilter(double q)
		: statedim(h.statedim)
		, measuredim(h.measuredim)
		, svdstate(h.statedim, h.statedim)
		, svdinnovation(h.measuredim, h.measuredim)
	{
		sigmadim = 2 * statedim + 1;

		// ukf parameters
		// see https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf
		ap = 0.01;
		bt = 2.0;
		kp = 0.01;
		ld = ap * ap * (statedim + kp) - statedim;

		wm = std::vector<double>(sigmadim, 1.0 / (2.0 * (statedim + ld)));
		wc = std::vector<double>(sigmadim, 1.0 / (2.0 * (statedim + ld)));
		wm[0] = ld / (statedim + ld);
		wc[0] = ld / (statedim + ld) + (1 - ap * ap + bt);

		// set the measurement
		// noise for the motion model
		f.q = q;

		int i;
		// N is state dimension
		// M is measurement dimension
		// state is Nx1 and sigma is NxN
		state = std::vector<double>(statedim, 0.0);
		state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
		state_unc_sqrtm = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
		for(i = 0; i < statedim; i++)
		{
			state_unc[i][i] = 10.0;
		}
		state_sigma_points = std::vector<std::vector<double>>(sigmadim, std::vector<double>(statedim, 0.0));

		// expected measurement vector and measurement sigma points
		measurement = std::vector<double>(measuredim, 0.0);
		measurement_sigma_points =
			std::vector<std::vector<double>>(sigmadim, std::vector<double>(measuredim, 0.0));

		// process and measurement uncertainty matrices
		process_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
		measure_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

		// innovation is Nx1
		innovation = std::vector<double>(measuredim, 0.0);
		innovation_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
		innovation_unc_inv =
			std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

		// measurement and state covariance
		state_measurement_cov =
			std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

		// kalman gain is NxM
		gain = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));
		gainT = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));

		// update variables
		dx = std::vector<double>(statedim, 0.0);

		tmp = std::vector<std::vector<double>>(
			MAX(measuredim, statedim), std::vector<double>(MAX(measuredim, statedim), 0.0));
		tmpunc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	}

	void predict(double delta)
	{
		int i;
		f.getProcessUncertainty(delta, process_unc);

		// calculate sigma points
		svdstate.dcmp(state_unc);
		svdstate.sqrtm(state_unc_sqrtm);

		state_sigma_points[0] = state;
		for(i = 0; i < statedim; i++)
		{
			linalg::vecmult(sqrt(statedim + ld), state_unc_sqrtm[i], state_unc_sqrtm[i], statedim);
			linalg::vecadd(state, state_unc_sqrtm[i], state_sigma_points[i + 1], statedim);
			linalg::vecsubtract(state, state_unc_sqrtm[i], state_sigma_points[i + statedim + 1], statedim);
		}

		// make prediction for each sigma point
		for(i = 0; i < sigmadim; i++)
		{
			f.predict(delta, state_sigma_points[i]);
		}

		linalg::weightedsum(wm, state_sigma_points, state, sigmadim, statedim);
		linalg::weightedmult(
			wc,
			state_sigma_points,
			state,
			state_sigma_points,
			state,
			state_unc,
			sigmadim,
			statedim,
			statedim);
		linalg::matadd(state_unc, process_unc, state_unc, statedim, statedim);
	}

	template <typename Z>
	void update(Z& z)
	{
		h.getMeasurementUncertainty(z, measure_unc);
		for(int i = 0; i < sigmadim; i++)
		{
			h.predict(state_sigma_points[i], z, measurement_sigma_points[i]);
		}
		linalg::weightedsum(wm, measurement_sigma_points, measurement, sigmadim, measuredim);
		linalg::weightedmult(
			wc,
			measurement_sigma_points,
			measurement,
			measurement_sigma_points,
			measurement,
			innovation_unc,
			sigmadim,
			measuredim,
			measuredim);
		linalg::matadd(innovation_unc, measure_unc, innovation_unc, measuredim, measuredim);

		svdinnovation.dcmp(innovation_unc);
		svdinnovation.inverse(innovation_unc_inv);

		linalg::weightedmult(
			wc,
			state_sigma_points,
			state,
			measurement_sigma_points,
			measurement,
			state_measurement_cov,
			sigmadim,
			statedim,
			measuredim);

		linalg::matmult(state_measurement_cov, innovation_unc_inv, gain, statedim, measuredim, measuredim);

		// update state
		// TODO: make this generic
		innovation[0] = z.x - measurement[0];
		innovation[1] = z.y - measurement[1];
		innovation[2] = z.z - measurement[2];
		linalg::matvecmult(gain, innovation, dx, statedim, measuredim);
		linalg::vecadd(state, dx, state, statedim);

		// update covariance
		linalg::transpose(gain, gainT, statedim, measuredim);
		linalg::matmult(innovation_unc, gainT, tmp, measuredim, measuredim, statedim);
		linalg::matmult(gain, tmp, tmpunc, statedim, measuredim, statedim);
		linalg::matsubtract(state_unc, tmpunc, state_unc, statedim, statedim);
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

	// temp vectors/matrices for calculations
	std::vector<std::vector<double>> innovation_unc_inv;
	std::vector<double> dx;
	std::vector<std::vector<double>> eye, tmp, tmpunc;

	// matrix inversion and sigma points
	linalg::SVD svdinnovation;
	linalg::SVD svdstate;
};

#endif /* _UKF_HPP_ */