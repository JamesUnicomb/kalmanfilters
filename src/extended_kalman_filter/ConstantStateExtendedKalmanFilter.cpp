#include "ConstantStateExtendedKalmanFilter.hpp"
#include "linalg/linalg.hpp"

using namespace std;
using namespace linalg;

ConstantStateExtendedKalmanFilter::ConstantStateExtendedKalmanFilter(
	double process_unc, double measurement_unc)
	: process_unc(process_unc)
	, measurement_unc(measurement_unc)
{
	// state is 2x1 and sigma is 2x2
	state = std::vector<double>(statedim, 0.0);
	state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	state_unc[0][0] = state_unc[1][1] = 1.0;

	// innovation is 3x1
	innovation = std::vector<double>(measuredim, 0.0);
	innovation_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
	innovation_unc_inv = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

	// fill tmp matrices
	jac = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));
	jacT = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

	// kalman gain is 2x3
	gain = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

	// update variables
	dx = std::vector<double>(statedim, 0.0);
	eye = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	eye[0][0] = eye[1][1] = 1.0;

	tmp = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
	tmpunc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
}

void ConstantStateExtendedKalmanFilter::update(const vector<double>& accel)
{
	// run trig functions once
	double sr, cr, cp, sp;
	sr = sin(state[0]);
	cr = cos(state[0]);
	sp = sin(state[1]);
	cp = cos(state[1]);

	// calculate innovation
	// y = z - h(x)
	innovation[0] = accel[0] - sp * g;
	innovation[1] = accel[1] + sr * cp * g;
	innovation[2] = accel[2] - cr * cp * g;

	// innovation uncertainty
	// S = jac/dx * sigma * jac/dx^T + R
	jac[0][0] = 0.0;
	jac[0][1] = cp * g;
	jac[1][0] = -cr * cp * g;
	jac[1][1] = sr * sp * g;
	jac[2][0] = -sr * cp * g;
	jac[2][1] = -cr * sp * g;

	transpose(jac, jacT, 3, 2);
	matmult(jac, state_unc, tmp, 3, 2, 2);
	matmult(tmp, jacT, innovation_unc, 3, 2, 3);
	for(int i = 0; i < 3; i++)
	{
		innovation_unc[i][i] += measurement_unc;
	}

	// calculate kalman gain
	cholesky cho(innovation_unc);
	cho.inverse(innovation_unc_inv);
	matmult(jacT, innovation_unc_inv, tmp, 2, 3, 3);
	matmult(state_unc, tmp, gain, 2, 2, 3);

	// update
	matvecmult(gain, innovation, dx, 2, 3);
	for(int i = 0; i < 2; i++)
	{
		state[i] += dx[i];
	}

	matmult(gain, jac, tmp, 2, 3, 2);
	matsubtract(eye, tmp, tmp, 2, 2);
	matcopy(state_unc, tmpunc, 2, 2);
	matmult(tmp, tmpunc, state_unc, 2, 2, 2);
}

void ConstantStateExtendedKalmanFilter::predict(double dt)
{
	// x = f(x) ~ x
	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	for(int i = 0; i < 2; i++)
	{
		state_unc[i][i] += dt * process_unc;
	}
}