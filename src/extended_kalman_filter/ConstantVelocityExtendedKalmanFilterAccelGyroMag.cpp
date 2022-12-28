#include "ConstantVelocityExtendedKalmanFilterAccelGyroMag.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

using namespace std;
using namespace linalg;

Tracking::Tracking(double process_unc)
	: process_unc(process_unc)
{
	int i;
	// state is 6x1 and sigma is 6x6
	state = std::vector<double>(statedim, 0.0);
	state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	for(i = 0; i < statedim; i++)
	{
		state_unc[i][i] = 10.0;
	}

	// innovation is 6x1
	innovation = std::vector<double>(measuredim, 0.0);
	innovation_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
	innovation_unc_inv = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
	measure_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

	// fill tmp matrices
	jac = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));
	jacT = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

	// kalman gain is 6x3
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

void Tracking::update(const sensors::accel& accel, double unc)
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
	jac[0][4] = 0.0;
	jac[0][5] = 0.0;
	jac[1][0] = c0 * c1 * g;
	jac[1][1] = -s0 * s1 * g;
	jac[1][2] = 0.0;
	jac[1][3] = 0.0;
	jac[1][4] = 0.0;
	jac[1][5] = 0.0;
	jac[2][0] = -s0 * c1 * g;
	jac[2][1] = -c0 * s1 * g;
	jac[2][2] = 0.0;
	jac[2][3] = 0.0;
	jac[2][4] = 0.0;
	jac[2][5] = 0.0;

	for(int i = 0; i < measuredim; i++)
	{
		measure_unc[i][i] = unc;
	}

    update();
}

void Tracking::update(const sensors::gyro& gyro, double unc)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate innovation
	// y = z - h(x)
	innovation[0] = gyro.x - (state[3] + s1 * state[5]);
	innovation[1] = gyro.y - (c0 * state[4] + s0 * c1 * state[5]);
	innovation[2] = gyro.z - (-s0 * state[4] + c0 * c1 * state[5]);

	// innovation uncertainty
	// S = jac/dx * sigma * jac/dx^T + R
	jac[0][0] = 0.0;
	jac[0][1] = c1 * state[5];
	jac[0][2] = 0.0;
	jac[0][3] = 1.0;
	jac[0][4] = 0.0;
	jac[0][5] = s1;

	jac[1][0] = -s0 * state[4] + c0 * c1 * state[5];
	jac[1][1] = -s0 * s1 * state[5];
	jac[1][2] = 0.0;
	jac[1][3] = 0.0;
	jac[1][4] = c0;
	jac[1][5] = s0 * c1;

	jac[2][0] = -c0 * state[4] - s0 * c1 * state[5];
	jac[2][1] = -c0 * s1 * state[5];
	jac[2][2] = 0.0;
	jac[2][3] = 0.0;
	jac[2][4] = c0;
	jac[2][5] = c0 * c1;

	for(int i = 0; i < measuredim; i++)
	{
		measure_unc[i][i] = unc;
	}
    
	update();
}

void Tracking::update(const sensors::mag& mag, double unc)
{
	// run trig functions once
	double s0, c0, s1, c1, s2, c2;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);
	s2 = sin(state[2]);
	c2 = cos(state[2]);

	double mx, my, mz;
	mx = 24.0475;
	my = 5.4344;
	mz = 51.4601;

	// calculate innovation
	// y = z - h(x)
	innovation[0] = mag.x - (c2 * c1 * mx + s2 * c1 * my - s1 * mz);
	innovation[1] = mag.y - ((c2 * s1 * s0 - s2 * c0) * mx + (s2 * s1 * s0 + c2 * c0) * my + c1 * s0 * mz);
	innovation[2] = mag.z - ((c2 * s1 * c0 + s2 * s0) * mx + (s2 * s1 * c0 - c2 * s0) * my + c1 * c0 * mz);

	// innovation uncertainty
	// S = jac/dx * sigma * jac/dx^T + R
	jac[0][0] = 0.0;
	jac[0][1] = -mx * s1 * c2 - my * s2 * s1 - mz * c1;
	jac[0][2] = -mx * s2 * c1 + my * c2 * c1;
	jac[0][3] = 0.0;
	jac[0][4] = 0.0;
	jac[0][5] = 0.0;

	jac[1][0] = mx * (s0 * s2 + s1 * c0 * c2) + my * (-s0 * c2 + s2 * s1 * c0) + mz * c0 * c1;
	jac[1][1] = mx * s0 * c2 * c1 + my * s0 * s2 * c1 - mz * s0 * s1;
	jac[1][2] = mx * (-s0 * s2 * s1 - c0 * c2) + my * (s0 * s1 * c2 - s2 * c0);
	jac[1][3] = 0.0;
	jac[1][4] = 0.0;
	jac[1][5] = 0.0;

	jac[2][0] = mx * (-s0 * s1 * c2 + s2 * c0) + my * (-s0 * s2 * s1 - c0 * c2) - mz * s0 * c1;
	jac[2][1] = mx * c0 * c2 * c1 + my * s2 * c0 * c1 - mz * s1 * c0;
	jac[2][2] = mx * (s0 * c2 - s2 * s1 * c0) + my * (s0 * s2 + s1 * c0 * c2);
	jac[2][3] = 0.0;
	jac[2][4] = 0.0;
	jac[2][5] = 0.0;

	for(int i = 0; i < measuredim; i++)
	{
		measure_unc[i][i] = unc;
	}

	update();
}

void Tracking::update() {
	transpose(jac, jacT, measuredim, statedim);
	matmult(jac, state_unc, tmp, measuredim, statedim, statedim);
	matmult(tmp, jacT, innovation_unc, measuredim, statedim, measuredim);
	matadd(innovation_unc, measure_unc, innovation_unc, measuredim, measuredim);

	// calculate kalman gain
	cholesky cho(innovation_unc);
	cho.inverse(innovation_unc_inv);
	matmult(jacT, innovation_unc_inv, tmp, statedim, measuredim, measuredim);
	matmult(state_unc, tmp, gain, statedim, statedim, measuredim);

	// update
	matvecmult(gain, innovation, dx, statedim, measuredim);
	vecadd(state, dx, state, statedim);

	matmult(gain, jac, tmp, statedim, measuredim, statedim);
	matsubtract(eye, tmp, tmp, statedim, statedim);
	matcopy(state_unc, tmpunc, statedim, statedim);
	matmult(tmp, tmpunc, state_unc, statedim, statedim, statedim);
}

void Tracking::predict(double dt)
{
	int i;
	// x = f(x) ~ [1, dt] * x
	state[0] += dt * state[3];
	state[1] += dt * state[4];
	state[2] += dt * state[5];

	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	// compute df/dx
	setzero(tmp, statedim, statedim);
	for(i = 0; i < statedim; i++)
	{
		tmp[i][i] = 1.0;
	}
	tmp[0][3] = dt;
	tmp[1][4] = dt;
	tmp[2][5] = dt;

	// compute df/dx * sigma * df/dx^T
	matmult(tmp, state_unc, tmpunc, statedim, statedim, statedim);
	tmp[0][3] = 0.0;
	tmp[3][0] = dt; // transpose df/dx
	tmp[1][4] = 0.0;
	tmp[4][1] = dt; // transpose df/dx
	tmp[2][5] = 0.0;
	tmp[5][2] = dt; // transpose df/dx
	matmult(tmpunc, tmp, state_unc, statedim, statedim, statedim);

	// add Q
	for(i = 0; i < 3; i++)
	{
		state_unc[i][i] += dt * dt * dt / 3.0 * process_unc;
		state_unc[i + 3][i] += dt * dt / 2.0 * process_unc;
		state_unc[i][i + 3] += dt * dt / 2.0 * process_unc;
		state_unc[i + 3][i + 3] += dt * process_unc;
	}
}