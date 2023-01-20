#ifndef _EKF_HPP_
#define _EKF_HPP_

#include "nr3.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"
#include <vector>
#include <cmath>

template <typename F, typename H>
class ExtendedKalmanFilter
{
public:
	/*
    x = f(x,dt)
    sigma = df/dx * sigma * df/dx^T + Q

    y = z - h(x)
    S = dh/dx * sigma * dh/dx^T + R
    K = dh/dx * sigma * S^-1

    x = x + K * y
    sigma = (I - K * dh/dx) * sigma
    */

	ExtendedKalmanFilter(double q, linalg::Vector state, linalg::Matrix state_unc)
		: state(state)
		, state_unc(state_unc)
		, statedim(h.statedim)
		, measuredim(h.measuredim)
		, svd(h.measuredim, h.measuredim)
	{
		int i;

		// set motion uncertainty
		f.q = q;

		// process and measurement uncertainty matrices
		process_unc = linalg::Matrix(statedim, statedim, 0.0);
		measure_unc = linalg::Matrix(measuredim, measuredim, 0.0);

		// innovation is Nx1
		innovation = linalg::Vector(measuredim, 0.0);
		innovation_unc = linalg::Matrix(measuredim, measuredim, 0.0);
		innovation_unc_inv = linalg::Matrix(measuredim, measuredim, 0.0);

		// fill tmp matrices
		dfdx = linalg::Matrix(statedim, statedim, 0.0);
		dfdxT = linalg::Matrix(statedim, statedim, 0.0);
		dhdx = linalg::Matrix(measuredim, statedim, 0.0);
		dhdxT = linalg::Matrix(statedim, measuredim, 0.0);

		// kalman gain is NxM
		gain = linalg::Matrix(statedim, measuredim, 0.0);

		// update variables
		dx = linalg::Vector(statedim, 0.0);
		eye = linalg::Matrix(statedim, statedim, 0.0);
		for(i = 0; i < statedim; i++)
		{
			eye[i][i] = 1.0;
		}

		// temporary matrices
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
		f(delta, state, dfdx, process_unc);

		linalg::transpose(dfdx, dfdxT);

		linalg::mult(dfdx, state_unc, tmpunc);
		linalg::mult(tmpunc, dfdxT, state_unc);

		linalg::add(state_unc, process_unc, state_unc);

		f.final(state, state_unc);
	}

	template <typename Z>
	void update(Z& z)
	{
		h(state, z, innovation, dhdx, measure_unc);

		linalg::transpose(dhdx, dhdxT);
		linalg::mult(dhdx, state_unc, tmpmn);
		linalg::mult(tmpmn, dhdxT, innovation_unc);
		innovation_unc += measure_unc;

		// calculate kalman gain
		svd.dcmp(innovation_unc);
		svd.inverse(innovation_unc_inv);
		linalg::mult(dhdxT, innovation_unc_inv, tmpnm);
		linalg::mult(state_unc, tmpnm, gain);

		// update state
		linalg::mult(gain, innovation, dx);
		state += dx;

		// update covariance
		linalg::mult(gain, dhdx, tmpnn);
		linalg::subtract(eye, tmpnn, tmpnn);
		tmpunc = state_unc;
		linalg::mult(tmpnn, tmpunc, state_unc);

		// update normalization routine
		h.final(state, state_unc);
	}

private:
	F f;
	H h;

	// x and sigma
	linalg::Vector state;
	linalg::Matrix state_unc;

	// df/dx, df/dx^T, dh/dx, dh/dx^T
	linalg::Matrix dfdx, dfdxT, dhdx, dhdxT;

	// Q and R
	linalg::Matrix process_unc, measure_unc;

	// y and S
	linalg::Vector innovation;
	linalg::Matrix innovation_unc;

	// K
	linalg::Matrix gain;

	// temp vectors/matrices for calculations
	linalg::Matrix innovation_unc_inv;
	linalg::Vector dx;
	linalg::Matrix eye;
	linalg::Matrix tmpunc;
	linalg::Matrix tmpnn, tmpnm, tmpmn, tmpmm;

	// model and measurement dimension
	const int statedim;
	const int measuredim;

	// matrix inversion
	linalg::SVD svd;
};

#endif /* _EKF_HPP_ */