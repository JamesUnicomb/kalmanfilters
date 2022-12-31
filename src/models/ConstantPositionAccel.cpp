#include "ConstantPositionAccel.hpp"
#include "linalg/linalg.hpp"

using namespace std;
using namespace linalg;

void ConstantPositionAccelMotionModel::predict(double delta, std::vector<double>& state)
{
	// x = f(x) ~ x
}

void ConstantPositionAccelMotionModel::derivs(
	double delta, std::vector<double>& state, std::vector<std::vector<double>>& jac)
{
	jac[0][0] = jac[1][1] = 1.0;
	jac[1][0] = jac[0][1] = 0.0;
}

void ConstantPositionAccelMotionModel::getProcessUncertainty(
	double delta, std::vector<std::vector<double>>& process_unc)
{
	// x = f(x) ~ x
	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	for(int i = 0; i < 2; i++)
	{
		process_unc[i][i] = delta * q;
	}
}

void ConstantPositionAccelMotionModel::operator()(
	double delta,
	std::vector<double>& state,
	std::vector<std::vector<double>>& jac,
	std::vector<std::vector<double>>& process_unc)
{
	predict(delta, state);
	derivs(delta, state, jac);
	getProcessUncertainty(delta, process_unc);
}

void ConstantPositionAccelMeasurementModel::innovation(
	std::vector<double>& state, sensors::accel& accel, std::vector<double>& y)
{
	// run trig functions once
	double s0, c0, c1, s1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate innovation
	// y = z - h(x)
	y[0] = accel.x - (-s1 * g);
	y[1] = accel.y - (s0 * c1 * g);
	y[2] = accel.z - (c0 * c1 * g);
}

void ConstantPositionAccelMeasurementModel::operator()(
	std::vector<double>& state,
	sensors::accel& accel,
	std::vector<double>& y,
	std::vector<std::vector<double>>& jac,
	std::vector<std::vector<double>>& measure_unc)
{
	innovation(state, accel, y);
	derivs(state, accel, jac);
	getMeasurementUncertainty(accel, measure_unc);
}

void ConstantPositionAccelMeasurementModel::derivs(
	std::vector<double>& state, sensors::accel& accel, std::vector<std::vector<double>>& jac)
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

void ConstantPositionAccelMeasurementModel::getMeasurementUncertainty(
	sensors::accel& accel, std::vector<std::vector<double>>& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = accel.xunc;
	measure_unc[1][1] = accel.yunc;
	measure_unc[2][2] = accel.zunc;
}