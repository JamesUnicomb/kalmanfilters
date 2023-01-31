#include "ConstantPositionAccelMagQuat.hpp"
#include "linalg/linalg.hpp"

void ConstantPositionAccelMagQuatMotionModel::predict(double delta, linalg::Vector& state)
{
	// x = f(x) ~ x
}

void ConstantPositionAccelMagQuatMotionModel::derivs(double delta, linalg::Vector& state, linalg::Matrix& jac)
{
	jac[0][0] = jac[1][1] = jac[2][2] = jac[3][3] = 1.0;
	jac[0][1] = jac[0][2] = jac[0][3] = 0.0;
	jac[1][0] = jac[1][2] = jac[1][3] = 0.0;
	jac[2][0] = jac[2][1] = jac[2][3] = 0.0;
	jac[3][0] = jac[3][1] = jac[3][2] = 0.0;
}

void ConstantPositionAccelMagQuatMotionModel::getProcessUncertainty(
	double delta, linalg::Vector& state, linalg::Matrix& process_unc)
{
	// x = f(x) ~ x
	// sigma = df/dx * sigma * df/dx^T + Q ~ sigma + Q
	for(int i = 0; i < 4; i++)
	{
		process_unc[i][i] = delta * q_;
	}
}

void ConstantPositionAccelMagQuatMotionModel::operator()(
	double delta, linalg::Vector& state, linalg::Matrix& jac, linalg::Matrix& process_unc)
{
	derivs(delta, state, jac);
	getProcessUncertainty(delta, state, process_unc);
	predict(delta, state);
}

template <typename M>
void ConstantPositionAccelMagQuatMeasurementModel<M>::predict(
	linalg::Vector& state, sensors::accel& accel, linalg::Vector& y)
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

template <typename M>
void ConstantPositionAccelMagQuatMeasurementModel<M>::predict(
	linalg::Vector& state, sensors::mag& mag, linalg::Vector& y)
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

template <typename M>
void ConstantPositionAccelMagQuatMeasurementModel<M>::innovation(
	linalg::Vector& state, sensors::accel& accel, linalg::Vector& y)
{
	// calculate innovation
	// y = h(x)
	predict(state, accel, y);
	// y = z - y = z - h(x)
	subtract(accel.vec(), y, y);
}

template <typename M>
void ConstantPositionAccelMagQuatMeasurementModel<M>::innovation(
	linalg::Vector& state, sensors::mag& mag, linalg::Vector& y)
{
	// calculate innovation
	// y = h(x)
	predict(state, mag, y);
	// y = z - y = z - h(x)
	subtract(mag.vec(), y, y);
}

template <typename M>
void ConstantPositionAccelMagQuatMeasurementModel<M>::operator()(
	linalg::Vector& state, sensors::accel& accel, linalg::Vector& y, linalg::Matrix& jac)
{
	innovation(state, accel, y);
	derivs(state, accel, jac);
}

template <typename M>
void ConstantPositionAccelMagQuatMeasurementModel<M>::operator()(
	linalg::Vector& state, sensors::mag& mag, linalg::Vector& y, linalg::Matrix& jac)
{
	innovation(state, mag, y);
	derivs(state, mag, jac);
}

template <typename M>
void ConstantPositionAccelMagQuatMeasurementModel<M>::derivs(
	linalg::Vector& state, sensors::accel& accel, linalg::Matrix& jac)
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

template <typename M>
void ConstantPositionAccelMagQuatMeasurementModel<M>::derivs(
	linalg::Vector& state, sensors::mag& mag, linalg::Matrix& jac)
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