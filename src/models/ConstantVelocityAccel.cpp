#include "ConstantVelocityAccel.hpp"
#include "linalg/linalg.hpp"

using namespace std;
using namespace linalg;

void ConstantVelocityAccelMotionModel::predict(double delta, vector<double>& state)
{
	// x = f(x) ~ [1, dt] * x
	state[0] += delta * state[2];
	state[1] += delta * state[3];
}

void ConstantVelocityAccelMotionModel::derivs(
	double delta, vector<double>& state, vector<vector<double>>& jac)
{
	int i;
	// df / dx = [1, dt]
	setzero(jac, statedim, statedim);
	for(i = 0; i < statedim; i++)
	{
		jac[i][i] = 1.0;
	}
	jac[0][2] = delta;
	jac[1][3] = delta;
}

void ConstantVelocityAccelMotionModel::getProcessUncertainty(
	double delta, vector<vector<double>>& process_unc)
{
	int i;
	// see https://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf
	// pages 12-13 for a derivation of the process noise
	// the model assumes changes in velocity are i.i.d white noise
	setzero(process_unc, statedim, statedim);
	for(i = 0; i < 2; i++)
	{
		process_unc[i][i] = delta * delta * delta / 3.0 * q;
		process_unc[i + 2][i] = delta * delta / 2.0 * q;
		process_unc[i][i + 2] = delta * delta / 2.0 * q;
		process_unc[i + 2][i + 2] = delta * q;
	}
}

void ConstantVelocityAccelMotionModel::operator()(
	double delta,
	std::vector<double>& state,
	std::vector<std::vector<double>>& jac,
	std::vector<std::vector<double>>& process_unc)
{
	predict(delta, state);
	derivs(delta, state, jac);
	getProcessUncertainty(delta, process_unc);
}

void ConstantVelocityAccelMeasurementModel::innovation(
	vector<double>& state, sensors::accel& accel, vector<double>& y)
{
	// run trig functions once
	double s0, c0, s1, c1;
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

void ConstantVelocityAccelMeasurementModel::operator()(
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

void ConstantVelocityAccelMeasurementModel::derivs(
	vector<double>& state, sensors::accel& accel, vector<vector<double>>& jac)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// H = dh/dx
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
}

void ConstantVelocityAccelMeasurementModel::getMeasurementUncertainty(
	sensors::accel& accel, vector<vector<double>>& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = accel.xunc;
	measure_unc[1][1] = accel.yunc;
	measure_unc[2][2] = accel.zunc;
}