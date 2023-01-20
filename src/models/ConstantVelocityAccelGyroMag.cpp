#include "ConstantVelocityAccelGyroMag.hpp"
#include "linalg/linalg.hpp"

using namespace std;
using namespace linalg;

void ConstantVelocityAccelGyroMagMotionModel::predict(double delta, Vector& state)
{
	// x = f(x) ~ [1, dt] * x
	state[0] += delta * state[3];
	state[1] += delta * state[4];
	state[2] += delta * state[5];
}

void ConstantVelocityAccelGyroMagMotionModel::derivs(double delta, Vector& state, Matrix& jac)
{
	int i;
	// df / dx = [1, dt]
	setzero(jac);
	for(i = 0; i < statedim; i++)
	{
		jac[i][i] = 1.0;
	}
	jac[0][3] = delta;
	jac[1][4] = delta;
	jac[2][5] = delta;
}

void ConstantVelocityAccelGyroMagMotionModel::getProcessUncertainty(double delta, Matrix& process_unc)
{
	int i;
	// see https://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf
	// pages 12-13 for a derivation of the process noise
	// the model assumes changes in velocity are i.i.d white noise
	setzero(process_unc);
	for(i = 0; i < 3; i++)
	{
		process_unc[i][i] = delta * delta * delta / 3.0 * q;
		process_unc[i + 3][i] = delta * delta / 2.0 * q;
		process_unc[i][i + 3] = delta * delta / 2.0 * q;
		process_unc[i + 3][i + 3] = delta * q;
	}
}

void ConstantVelocityAccelGyroMagMotionModel::operator()(
	double delta, Vector& state, Matrix& jac, Matrix& process_unc)
{
	derivs(delta, state, jac);
	getProcessUncertainty(delta, process_unc);
	predict(delta, state);
}

void ConstantVelocityAccelGyroMagMeasurementModel::predict(Vector& state, sensors::accel& accel, Vector& h)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate expected measurement
	// h(x)
	h[0] = -s1 * g;
	h[1] = s0 * c1 * g;
	h[2] = c0 * c1 * g;
}

void ConstantVelocityAccelGyroMagMeasurementModel::predict(Vector& state, sensors::gyro& gyro, Vector& h)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate expected measurement
	// h(x)
	h[0] = state[3] + s1 * state[5];
	h[1] = c0 * state[4] + s0 * c1 * state[5];
	h[2] = -s0 * state[4] + c0 * c1 * state[5];
}

void ConstantVelocityAccelGyroMagMeasurementModel::predict(Vector& state, sensors::mag& mag, Vector& h)
{
	// run trig functions once
	double s0, c0, s1, c1, s2, c2;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);
	s2 = sin(state[2]);
	c2 = cos(state[2]);

	// calculate expected measurement
	// h(x)
	h[0] = c2 * c1 * mx + s2 * c1 * my - s1 * mz;
	h[1] = (c2 * s1 * s0 - s2 * c0) * mx + (s2 * s1 * s0 + c2 * c0) * my + c1 * s0 * mz;
	h[2] = (c2 * s1 * c0 + s2 * s0) * mx + (s2 * s1 * c0 - c2 * s0) * my + c1 * c0 * mz;
}

void ConstantVelocityAccelGyroMagMeasurementModel::innovation(Vector& state, sensors::accel& accel, Vector& y)
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

void ConstantVelocityAccelGyroMagMeasurementModel::innovation(Vector& state, sensors::gyro& gyro, Vector& y)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// calculate innovation
	// y = z - h(x)
	y[0] = gyro.x - (state[3] + s1 * state[5]);
	y[1] = gyro.y - (c0 * state[4] + s0 * c1 * state[5]);
	y[2] = gyro.z - (-s0 * state[4] + c0 * c1 * state[5]);
}

void ConstantVelocityAccelGyroMagMeasurementModel::innovation(Vector& state, sensors::mag& mag, Vector& y)
{
	// run trig functions once
	double s0, c0, s1, c1, s2, c2;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);
	s2 = sin(state[2]);
	c2 = cos(state[2]);

	// calculate innovation
	// y = z - h(x)
	y[0] = mag.x - (c2 * c1 * mx + s2 * c1 * my - s1 * mz);
	y[1] = mag.y - ((c2 * s1 * s0 - s2 * c0) * mx + (s2 * s1 * s0 + c2 * c0) * my + c1 * s0 * mz);
	y[2] = mag.z - ((c2 * s1 * c0 + s2 * s0) * mx + (s2 * s1 * c0 - c2 * s0) * my + c1 * c0 * mz);
}

void ConstantVelocityAccelGyroMagMeasurementModel::operator()(
	Vector& state, sensors::accel& accel, Vector& y, Matrix& jac, Matrix& measure_unc)
{
	innovation(state, accel, y);
	derivs(state, accel, jac);
	getMeasurementUncertainty(accel, measure_unc);
}

void ConstantVelocityAccelGyroMagMeasurementModel::operator()(
	Vector& state, sensors::gyro& gyro, Vector& y, Matrix& jac, Matrix& measure_unc)
{
	innovation(state, gyro, y);
	derivs(state, gyro, jac);
	getMeasurementUncertainty(gyro, measure_unc);
}

void ConstantVelocityAccelGyroMagMeasurementModel::operator()(
	Vector& state, sensors::mag& mag, Vector& y, Matrix& jac, Matrix& measure_unc)
{
	innovation(state, mag, y);
	derivs(state, mag, jac);
	getMeasurementUncertainty(mag, measure_unc);
}

void ConstantVelocityAccelGyroMagMeasurementModel::derivs(Vector& state, sensors::accel& accel, Matrix& jac)
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
}

void ConstantVelocityAccelGyroMagMeasurementModel::derivs(Vector& state, sensors::gyro& gyro, Matrix& jac)
{
	// run trig functions once
	double s0, c0, s1, c1;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);

	// H = dh/dx
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
}

void ConstantVelocityAccelGyroMagMeasurementModel::derivs(Vector& state, sensors::mag& mag, Matrix& jac)
{
	// run trig functions once
	double s0, c0, s1, c1, s2, c2;
	s0 = sin(state[0]);
	c0 = cos(state[0]);
	s1 = sin(state[1]);
	c1 = cos(state[1]);
	s2 = sin(state[2]);
	c2 = cos(state[2]);

	// H = dh/dx
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
}

void ConstantVelocityAccelGyroMagMeasurementModel::getMeasurementUncertainty(
	sensors::accel& accel, Matrix& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = accel.xunc;
	measure_unc[1][1] = accel.yunc;
	measure_unc[2][2] = accel.zunc;
}

void ConstantVelocityAccelGyroMagMeasurementModel::getMeasurementUncertainty(
	sensors::gyro& gyro, Matrix& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = gyro.xunc;
	measure_unc[1][1] = gyro.yunc;
	measure_unc[2][2] = gyro.zunc;
}

void ConstantVelocityAccelGyroMagMeasurementModel::getMeasurementUncertainty(
	sensors::mag& mag, Matrix& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = mag.xunc;
	measure_unc[1][1] = mag.yunc;
	measure_unc[2][2] = mag.zunc;
}
