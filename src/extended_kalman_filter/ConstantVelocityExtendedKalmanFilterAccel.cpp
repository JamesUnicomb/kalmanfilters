#include "ConstantVelocityExtendedKalmanFilterAccel.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

using namespace std;
using namespace linalg;

ConstantVelocityExtendedKalmanFilterAccel::ConstantVelocityExtendedKalmanFilterAccel(
	double process_unc)
	: process_unc(process_unc)
{
	int i;
	// state is 4x1 and sigma is 4x4
	state = std::vector<double>(statedim, 0.0);
	state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	for(i = 0; i < statedim; i++)
	{
		state_unc[i][i] = 1.0;
	}

	// innovation is 4x1
	innovation = std::vector<double>(measuredim, 0.0);
	innovation_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
	innovation_unc_inv = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

	// fill tmp matrices
	jac = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));
	jacT = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

	// kalman gain is 4x3
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

void ConstantVelocityExtendedKalmanFilterAccel::update(const sensors::accel& accel, double unc)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate innovation
	// y = z - h(x)
	innovation[0] = accel.x - (-s1 * g);
	innovation[1] = accel.y - (s0 * c1 * g);
	innovation[2] = accel.z - (c0 * c1 * g);

	// innovation uncertainty
	// S = jac/dx * sigma * jac/dx^T + R
	jac[0][0] = 0.0;
	jac[0][1] = -c1 * g;
	jac[0][2] = 0.0;
	jac[0][3] = 0.0;
	jac[1][0] = c0 * c1 * g;
	jac[1][1] = -s0 * s1 * g;
	jac[1][2] = 0.0;
	jac[1][3] = 0.0;
	jac[2][0] = -s0 * c1 * g;
	jac[2][1] = -c0 * s1 * g;
	jac[2][2] = 0.0;
	jac[2][3] = 0.0;

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

void ConstantVelocityExtendedKalmanFilterAccel::predict(double dt)
{
    int i;
	// x = f(x) ~ [1, dt] * x
    state[0] += dt * state[2];
    state[1] += dt * state[3];

	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
    // compute df/dx
    setzero(tmp, statedim, statedim);
    for (i = 0; i < statedim; i++) {
        tmp[i][i] = 1.0;
    }
    tmp[0][2] = dt;
    tmp[1][3] = dt;
	
    // compute df/dx * sigma * df/dx^T
	matmult(tmp, state_unc, tmpunc, statedim, statedim, statedim);
    tmp[0][2] = 0.0; tmp[2][0] = dt; // transpose df/dx
    tmp[1][3] = 0.0; tmp[3][1] = dt; // transpose df/dx
    matmult(tmpunc, tmp, state_unc, statedim, statedim, statedim);

    // add Q
    for(i = 0; i < 2; i++)
	{
		state_unc[i][i] += dt * dt * dt / 3.0 * process_unc;
		state_unc[i + 2][i] += dt * dt / 2.0 * process_unc;
		state_unc[i][i + 2] += dt * dt / 2.0 * process_unc;
        state_unc[i+2][i+2] += dt * process_unc;
	}
}