#include "ConstantPositionAccelMagQuat.hpp"
#include "linalg/linalg.hpp"

using namespace std;
using namespace linalg;

void ConstantPositionAccelMagQuatMotionModel::predict(double delta, Vector& state)
{
	// x = f(x) ~ x
}

void ConstantPositionAccelMagQuatMotionModel::derivs(double delta, Vector& state, Matrix& jac)
{
	jac[0][0] = jac[1][1] = jac[2][2] = jac[3][3] = 1.0;
	jac[0][1] = jac[0][2] = jac[0][3] = 0.0;
	jac[1][0] = jac[1][2] = jac[1][3] = 0.0;
	jac[2][0] = jac[2][1] = jac[2][3] = 0.0;
	jac[3][0] = jac[3][1] = jac[3][2] = 0.0;
}

void ConstantPositionAccelMagQuatMotionModel::getProcessUncertainty(
	double delta, Vector& state, Matrix& process_unc)
{
	// x = f(x) ~ x
	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	for(int i = 0; i < 4; i++)
	{
		process_unc[i][i] = delta * q;
	}
}

void ConstantPositionAccelMagQuatMotionModel::operator()(
	double delta, Vector& state, Matrix& jac, Matrix& process_unc)
{
	derivs(delta, state, jac);
	getProcessUncertainty(delta, state, process_unc);
	predict(delta, state);
}

void ConstantPositionAccelMagQuatMeasurementModel::predict(Vector& state, sensors::accel& accel, Vector& y)
{
	double qx, qy, qz, qw;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];

	// calculate innovation
	// y = h(x)
	y[0] = (g * (-2 * qw * qy + 2 * qx * qz));
	y[1] = (g * (2 * qw * qx + 2 * qy * qz));
	y[2] = (g * (2 * qw * qw + 2 * qz * qz - 1));
}

void ConstantPositionAccelMagQuatMeasurementModel::predict(Vector& state, sensors::mag& mag, Vector& y)
{
	double qx, qy, qz, qw;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];

	// calculate innovation
	// y = h(x)
	y[0] =
		(mx * (2 * qw * qw + 2 * qx * qx - 1) + my * (2 * qw * qz + 2 * qx * qy) +
		 mz * (-2 * qw * qy + 2 * qx * qz));
	y[1] =
		(mx * (-2 * qw * qz + 2 * qx * qy) + my * (2 * qw * qw + 2 * qy * qy - 1) +
		 mz * (2 * qw * qx + 2 * qy * qz));
	y[2] =
		(mx * (2 * qw * qy + 2 * qx * qz) + my * (-2 * qw * qx + 2 * qy * qz) +
		 mz * (2 * qw * qw + 2 * qz * qz - 1));
}

void ConstantPositionAccelMagQuatMeasurementModel::innovation(Vector& state, sensors::accel& accel, Vector& y)
{
	// calculate innovation
	// y = z - h(x)
	predict(state, accel, y);
	// y = z - y = z - h(x)
	subtract(accel.vector(), y, y);
}

void ConstantPositionAccelMagQuatMeasurementModel::innovation(Vector& state, sensors::mag& mag, Vector& y)
{
	// calculate innovation
	// y = h(x)
	predict(state, mag, y);
	// y = z - y = z - h(x)
	subtract(mag.vector(), y, y);
}

void ConstantPositionAccelMagQuatMeasurementModel::operator()(
	Vector& state, sensors::accel& accel, Vector& y, Matrix& jac, Matrix& measure_unc)
{
	innovation(state, accel, y);
	derivs(state, accel, jac);
	getMeasurementUncertainty(accel, measure_unc);
}

void ConstantPositionAccelMagQuatMeasurementModel::operator()(
	Vector& state, sensors::mag& mag, Vector& y, Matrix& jac, Matrix& measure_unc)
{
	innovation(state, mag, y);
	derivs(state, mag, jac);
	getMeasurementUncertainty(mag, measure_unc);
}

void ConstantPositionAccelMagQuatMeasurementModel::derivs(Vector& state, sensors::accel& accel, Matrix& jac)
{
	double qw, qx, qy, qz;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];

	jac[0][0] = -2 * g * qy;
	jac[1][0] = 2 * g * qx;
	jac[2][0] = 4 * g * qw;
	jac[0][1] = 2 * g * qz;
	jac[1][1] = 2 * g * qw;
	jac[2][1] = 0;
	jac[0][2] = -2 * g * qw;
	jac[1][2] = 2 * g * qz;
	jac[2][2] = 0;
	jac[0][3] = 2 * g * qx;
	jac[1][3] = 2 * g * qy;
	jac[2][3] = 4 * g * qz;
}

void ConstantPositionAccelMagQuatMeasurementModel::derivs(Vector& state, sensors::mag& mag, Matrix& jac)
{
	double qw, qx, qy, qz;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];

	jac[0][0] = 4 * mx * qw + 2 * my * qz - 2 * mz * qy;
	jac[1][0] = -2 * mx * qz + 4 * my * qw + 2 * mz * qx;
	jac[2][0] = 2 * mx * qy - 2 * my * qx + 4 * mz * qw;
	jac[0][1] = 4 * mx * qx + 2 * my * qy + 2 * mz * qz;
	jac[1][1] = 2 * mx * qy + 2 * mz * qw;
	jac[2][1] = 2 * mx * qz - 2 * my * qw;
	jac[0][2] = 2 * my * qx - 2 * mz * qw;
	jac[1][2] = 2 * mx * qx + 4 * my * qy + 2 * mz * qz;
	jac[2][2] = 2 * mx * qw + 2 * my * qz;
	jac[0][3] = 2 * my * qw + 2 * mz * qx;
	jac[1][3] = -2 * mx * qw + 2 * mz * qy;
	jac[2][3] = 2 * mx * qx + 2 * my * qy + 4 * mz * qz;
}

void ConstantPositionAccelMagQuatMeasurementModel::getMeasurementUncertainty(
	sensors::accel& accel, Matrix& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = accel.xunc;
	measure_unc[1][1] = accel.yunc;
	measure_unc[2][2] = accel.zunc;
}

void ConstantPositionAccelMagQuatMeasurementModel::getMeasurementUncertainty(
	sensors::mag& mag, Matrix& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = mag.xunc;
	measure_unc[1][1] = mag.yunc;
	measure_unc[2][2] = mag.zunc;
}