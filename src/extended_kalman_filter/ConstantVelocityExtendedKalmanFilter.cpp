#include "ConstantVelocityExtendedKalmanFilter.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

using namespace std;
using namespace linalg;

ConstantVelocityExtendedKalmanFilter::ConstantVelocityExtendedKalmanFilter(
	double process_unc)
	: process_unc(process_unc)
{
	int i;
	// state is 5x1 and sigma is 5x5
	state = std::vector<double>(statedim, 0.0);
	state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	for(i = 0; i < statedim; i++)
	{
		state_unc[i][i] = 1.0;
	}

	// innovation is 5x1
	innovation = std::vector<double>(measuredim, 0.0);
	innovation_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
	innovation_unc_inv = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

	// fill tmp matrices
	jac = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));
	jacT = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

	// kalman gain is 5x3
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

void ConstantVelocityExtendedKalmanFilter::update(const sensors::accel& accel, double unc)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate innovation
	// y = z - h(x)
	innovation[0] = accel.x - s1 * g;
	innovation[1] = accel.y + s0 * c1 * g;
	innovation[2] = accel.z - c0 * c1 * g;

	// innovation uncertainty
	// S = jac/dx * sigma * jac/dx^T + R
	jac[0][0] = 0.0;
	jac[0][1] = c1 * g;
	jac[0][2] = 0.0;
	jac[0][3] = 0.0;
	jac[0][4] = 0.0;
	jac[1][0] = -c0 * c1 * g;
	jac[1][1] = s0 * s1 * g;
	jac[1][2] = 0.0;
	jac[1][3] = 0.0;
	jac[1][4] = 0.0;
	jac[2][0] = -s0 * c1 * g;
	jac[2][1] = -c0 * s1 * g;
	jac[2][2] = 0.0;
	jac[2][3] = 0.0;
	jac[2][4] = 0.0;

	transpose(jac, jacT, measuredim, statedim);
	matmult(jac, state_unc, tmp, measuredim, statedim, statedim);
	matmult(tmp, jacT, innovation_unc, measuredim, statedim, measuredim);
	for(int i = 0; i < measuredim; i++)
	{
		innovation_unc[i][i] += unc;
	}

	// calculate kalman gain
	cholesky cho(innovation_unc);
	cho.inverse(innovation_unc_inv);
	matmult(jacT, innovation_unc_inv, tmp, statedim, measuredim, measuredim);
	matmult(state_unc, tmp, gain, statedim, statedim, measuredim);

	// update
	matvecmult(gain, innovation, dx, statedim, measuredim);
	for(int i = 0; i < statedim; i++)
	{
		state[i] += dx[i];
	}

	matmult(gain, jac, tmp, statedim, measuredim, statedim);
	matsubtract(eye, tmp, tmp, statedim, statedim);
	matcopy(state_unc, tmpunc, statedim, statedim);
	matmult(tmp, tmpunc, state_unc, statedim, statedim, statedim);
}

void ConstantVelocityExtendedKalmanFilter::update(const sensors::gyro& gyro, double unc)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate innovation
	// y = z - h(x)
	innovation[0] = gyro.x - (state[2] + s1 * state[4]);
	innovation[1] = gyro.y - (c0 * state[3] + s0 * c1 * state[4]);
	innovation[2] = gyro.z - (-s0 * state[3] + c0 * c1 * state[4]);

	// innovation uncertainty
	// S = jac/dx * sigma * jac/dx^T + R
	jac[0][0] = 0.0;
	jac[0][1] = c1 * state[4];
	jac[0][2] = 1.0;
	jac[0][3] = 0.0;
	jac[0][4] = s1;
	jac[1][0] = -s0 * state[3] + c0 * c1 * state[4];
	jac[1][1] = -s0 * s1 * state[4];
	jac[1][2] = 0.0;
	jac[1][3] = c0;
	jac[1][4] = s0 * c1;
	jac[2][0] = -c0 * state[3] - s0 * c1 * state[4];
	jac[2][1] = -c0 * s1 * state[4];
	jac[2][2] = 0.0;
	jac[2][3] = c0;
	jac[2][4] = c0 * c1;

	transpose(jac, jacT, measuredim, statedim);
	matmult(jac, state_unc, tmp, measuredim, statedim, statedim);
	matmult(tmp, jacT, innovation_unc, measuredim, statedim, measuredim);
	for(int i = 0; i < measuredim; i++)
	{
		innovation_unc[i][i] += unc;
	}

	// calculate kalman gain
	cholesky cho(innovation_unc);
	cho.inverse(innovation_unc_inv);
	matmult(jacT, innovation_unc_inv, tmp, statedim, measuredim, measuredim);
	matmult(state_unc, tmp, gain, statedim, statedim, measuredim);

	// update
	matvecmult(gain, innovation, dx, statedim, measuredim);
	for(int i = 0; i < statedim; i++)
	{
		state[i] += dx[i];
	}

	matmult(gain, jac, tmp, statedim, measuredim, statedim);
	matsubtract(eye, tmp, tmp, statedim, statedim);
	matcopy(state_unc, tmpunc, statedim, statedim);
	matmult(tmp, tmpunc, state_unc, statedim, statedim, statedim);
}

void ConstantVelocityExtendedKalmanFilter::predict(double dt)
{
	// x = f(x) ~ [1, dt] * x
	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	int i;
	for(i = 0; i < 2; i++)
	{
		state_unc[i][i] += dt * dt * dt / 3.0 * process_unc + 2.0 * dt * state_unc[i + 2][i];
		state_unc[i + 2][i] += dt * dt / 2.0 * process_unc;
		state_unc[i][i + 2] += dt * dt / 2.0 * process_unc;
	}

	for(i = 2; i < 5; i++)
	{
		state_unc[i][i] += dt * process_unc;
	}
}