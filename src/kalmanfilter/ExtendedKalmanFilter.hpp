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
    
	// x and sigma
	std::vector<double> state;
	std::vector<std::vector<double>> state_unc;

    // df/dx, df/dx^T, dh/dx, dh/dx^T
	std::vector<std::vector<double>> dfdx, dfdxT, dhdx, dhdxT;

    // Q and R
    std::vector<std::vector<double>> process_unc, measure_unc;

    // y and S
	std::vector<double> innovation;
	std::vector<std::vector<double>> innovation_unc;

    // K
	std::vector<std::vector<double>> gain;

	ExtendedKalmanFilter(double q) {
		statedim = h.statedim;
		measuredim = h.measuredim;
		
		// set the measurement 
		// noise for the motion model
		f.q = q;

		int i;
		// N is state dimension
		// M is measurement dimension
		// state is Nx1 and sigma is NxN
		state = std::vector<double>(statedim, 0.0);
		state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
		for(i = 0; i < statedim; i++)
		{
			state_unc[i][i] = 10.0;
		}

		// process and measurement uncertainty matrices
		process_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
		measure_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

		// innovation is Nx1
		innovation = std::vector<double>(measuredim, 0.0);
		innovation_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
		innovation_unc_inv = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

		// fill tmp matrices
		dfdx = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
		dfdxT = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
		dhdx = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));
		dhdxT = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

		// kalman gain is NxM
		gain = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

		// update variables
		dx = std::vector<double>(statedim, 0.0);
		eye = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
		for(i = 0; i < statedim; i++)
		{
			eye[i][i] = 1.0;
		}

		tmp = std::vector<std::vector<double>>(
			MAX(measuredim, statedim), std::vector<double>(MAX(measuredim, statedim), 0.0));
		tmpunc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	}

	void predict(double delta) {
		f(delta, state);
		f.derivs(delta, state, dfdx);
		f.getProcessUncertainty(delta, process_unc);
		
		linalg::transpose(dfdx, dfdxT, statedim, statedim);

		linalg::matmult(dfdx, state_unc, tmp, statedim, statedim, statedim);
		linalg::matmult(tmp, dfdxT, state_unc, statedim, statedim, statedim);

		linalg::matadd(state_unc, process_unc, state_unc, statedim, statedim);
	}

	template <typename Z>
	void update(Z& z) {
		h(state, z, innovation);
		h.derivs(state, z, dhdx);
		h.getMeasurementUncertainty(z, measure_unc);

		linalg::transpose(dhdx, dhdxT, measuredim, statedim);
		linalg::matmult(dhdx, state_unc, tmp, measuredim, statedim, statedim);
		linalg::matmult(tmp, dhdxT, innovation_unc, measuredim, statedim, measuredim);
		linalg::matadd(innovation_unc, measure_unc, innovation_unc, measuredim, measuredim);

		// calculate kalman gain
		linalg::cholesky cho(innovation_unc);
		cho.inverse(innovation_unc_inv);
		linalg::matmult(dhdxT, innovation_unc_inv, tmp, statedim, measuredim, measuredim);
		linalg::matmult(state_unc, tmp, gain, statedim, statedim, measuredim);

		// update state
		linalg::matvecmult(gain, innovation, dx, statedim, measuredim);
		linalg::vecadd(state, dx, state, statedim);

        // update covariance
		linalg::matmult(gain, dhdx, tmp, statedim, measuredim, statedim);
		linalg::matsubtract(eye, tmp, tmp, statedim, statedim);
		linalg::matcopy(state_unc, tmpunc, statedim, statedim);
		linalg::matmult(tmp, tmpunc, state_unc, statedim, statedim, statedim);
	}

private:
    F f;
	H h;

	int statedim;
	int measuredim;

    // temp vectors/matrices for calculations
	std::vector<std::vector<double>> innovation_unc_inv;
	std::vector<double> dx;
	std::vector<std::vector<double>> eye, tmp, tmpunc;
};

#endif /* _EKF_HPP_ */