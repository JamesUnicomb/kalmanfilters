#include "ConstantPositionAccel.hpp"
#include "linalg/linalg.hpp"

using namespace std;
using namespace linalg;

void ConstantPositionAccelMotionModel::predict(double delta, Vector& state)
{
	// x = f(x) ~ x
}

void ConstantPositionAccelMotionModel::derivs(double delta, Vector& state, Matrix& jac)
{
	jac[0][0] = jac[1][1] = 1.0;
	jac[1][0] = jac[0][1] = 0.0;
}

void ConstantPositionAccelMotionModel::getProcessUncertainty(double delta, Matrix& process_unc)
{
	// x = f(x) ~ x
	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	for(int i = 0; i < 2; i++)
	{
		process_unc[i][i] = delta * q;
	}
}

void ConstantPositionAccelMotionModel::operator()(
	double delta, Vector& state, Matrix& jac, Matrix& process_unc)
{
	derivs(delta, state, jac);
	getProcessUncertainty(delta, process_unc);
	predict(delta, state);
}

void ConstantPositionAccelMeasurementModel::predict(Vector& state, sensors::accel& accel, Vector& y)
{
	// run trig functions once
	double s0, c0, c1, s1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate innovation
	// y = h(x)
	y[0] = -s1 * g;
	y[1] = s0 * c1 * g;
	y[2] = c0 * c1 * g;
}

void ConstantPositionAccelMeasurementModel::innovation(Vector& state, sensors::accel& accel, Vector& y)
{
	// calculate innovation
	// y = h(x)
	predict(state, accel, y);
	// y = z - y = z - h(x)
	subtract(accel.vec(), y, y);
}

void ConstantPositionAccelMeasurementModel::operator()(
	Vector& state, sensors::accel& accel, Vector& y, Matrix& jac)
{
	innovation(state, accel, y);
	derivs(state, accel, jac);
}

void ConstantPositionAccelMeasurementModel::derivs(Vector& state, sensors::accel& accel, Matrix& jac)
{
	// run trig functions once
	double s0, c0, c1, s1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	jac[0][0] = 0.0;
	jac[0][1] = -c1 * g;
	jac[1][0] = c0 * c1 * g;
	jac[1][1] = -s0 * s1 * g;
	jac[2][0] = -s0 * c1 * g;
	jac[2][1] = -c0 * s1 * g;
}