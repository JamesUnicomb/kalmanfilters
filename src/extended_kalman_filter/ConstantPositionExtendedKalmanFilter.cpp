#include "ConstantPositionExtendedKalmanFilter.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

using namespace std;
using namespace linalg;

ConstantPositionExtendedKalmanFilter::ConstantPositionExtendedKalmanFilter(
	double process_unc, double measurement_unc)
	: process_unc(process_unc)
	, measurement_unc(measurement_unc)
{
	int i;
	// state is 2x1 and sigma is 2x2
	state = std::vector<double>(statedim, 0.0);
	state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	for (i = 0; i < statedim; i++) {
        state_unc[i][i] = 1.0;
	}

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
	for (i = 0; i < statedim; i++) {
        eye[i][i] = 1.0;
	}

	tmp = std::vector<std::vector<double>>(MAX(measuredim, statedim), std::vector<double>(MAX(measuredim, statedim), 0.0));
	tmpunc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
}

void ConstantPositionExtendedKalmanFilter::update(const sensors::accel& accel)
{
	// run trig functions once
	double sr, cr, cp, sp;
	sr = sin(state[0]);
	cr = cos(state[0]);
	sp = sin(state[1]);
	cp = cos(state[1]);

	// calculate innovation
	// y = z - h(x)
	innovation[0] = accel.x - sp * g;
	innovation[1] = accel.y + sr * cp * g;
	innovation[2] = accel.z - cr * cp * g;

	// innovation uncertainty
	// S = jac/dx * sigma * jac/dx^T + R
	jac[0][0] = 0.0;
	jac[0][1] = cp * g;
	jac[1][0] = -cr * cp * g;
	jac[1][1] = sr * sp * g;
	jac[2][0] = -sr * cp * g;
	jac[2][1] = -cr * sp * g;

	transpose(jac, jacT, measuredim, statedim);
	matmult(jac, state_unc, tmp, measuredim, statedim, statedim);
	matmult(tmp, jacT, innovation_unc, measuredim, statedim, measuredim);
	for(int i = 0; i < measuredim; i++)
	{
		innovation_unc[i][i] += measurement_unc;
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

void ConstantPositionExtendedKalmanFilter::predict(double dt)
{
	// x = f(x) ~ x
	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	for(int i = 0; i < 2; i++)
	{
		state_unc[i][i] += dt * process_unc;
	}
}