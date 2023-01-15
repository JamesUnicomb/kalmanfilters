#include "ConstantVelocityAccelGyroMagQuat.hpp"
#include "linalg/linalg.hpp"

using namespace std;
using namespace linalg;

void ConstantVelocityAccelGyroMagQuatMotionModel::predict(double delta, vector<double>& state)
{
	double qx, qy, qz, qw, omx, omy, omz;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];
	omx = state[4];
	omy = state[5];
	omz = state[6];

	// x = f(x) ~ [1, dt] * x
	state[0] += delta * 0.5 * (-omx * qx - omy * qy - omz * qz);
	state[1] += delta * 0.5 * (omx * qw + omz * qy - omy * qz);
	state[2] += delta * 0.5 * (omy * qw - omz * qx + omx * qz);
	state[3] += delta * 0.5 * (omz * qw + omy * qx - omx * qy);
}

void ConstantVelocityAccelGyroMagQuatMotionModel::derivs(
	double delta, vector<double>& state, vector<vector<double>>& jac)
{
	int i;
	double qx, qy, qz, qw, omx, omy, omz;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];
	omx = state[4];
	omy = state[5];
	omz = state[6];

	jac[0][0] = 1;
	jac[1][0] = 0.5 * delta * omx;
	jac[2][0] = 0.5 * delta * omy;
	jac[3][0] = 0.5 * delta * omz;
	jac[4][0] = 0;
	jac[5][0] = 0;
	jac[6][0] = 0;

	jac[0][1] = -0.5 * delta * omx;
	jac[1][1] = 1;
	jac[2][1] = -0.5 * delta * omz;
	jac[3][1] = 0.5 * delta * omy;
	jac[4][1] = 0;
	jac[5][1] = 0;
	jac[6][1] = 0;

	jac[0][2] = -0.5 * delta * omy;
	jac[1][2] = 0.5 * delta * omz;
	jac[2][2] = 1;
	jac[3][2] = -0.5 * delta * omx;
	jac[4][2] = 0;
	jac[5][2] = 0;
	jac[6][2] = 0;

	jac[0][3] = -0.5 * delta * omz;
	jac[1][3] = -0.5 * delta * omy;
	jac[2][3] = 0.5 * delta * omx;
	jac[3][3] = 1;
	jac[4][3] = 0;
	jac[5][3] = 0;
	jac[6][3] = 0;

	jac[0][4] = -0.5 * delta * qx;
	jac[1][4] = 0.5 * delta * qw;
	jac[2][4] = 0.5 * delta * qz;
	jac[3][4] = -0.5 * delta * qy;
	jac[4][4] = 1;
	jac[5][4] = 0;
	jac[6][4] = 0;

	jac[0][5] = -0.5 * delta * qy;
	jac[1][5] = -0.5 * delta * qz;
	jac[2][5] = 0.5 * delta * qw;
	jac[3][5] = 0.5 * delta * qx;
	jac[4][5] = 0;
	jac[5][5] = 1;
	jac[6][5] = 0;

	jac[0][6] = -0.5 * delta * qz;
	jac[1][6] = 0.5 * delta * qy;
	jac[2][6] = -0.5 * delta * qx;
	jac[3][6] = 0.5 * delta * qw;
	jac[4][6] = 0;
	jac[5][6] = 0;
	jac[6][6] = 1;
}

void ConstantVelocityAccelGyroMagQuatMotionModel::getProcessUncertainty(
	double delta, vector<double>& state, vector<vector<double>>& process_unc)
{
	// see https://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf
	// pages 12-13 for a derivation of the process noise
	// the model assumes changes in velocity are i.i.d white noise
	double qx, qy, qz, qw, omx, omy, omz;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];
	omx = state[4];
	omy = state[5];
	omz = state[6];

	process_unc[0][0] = q / 3.0 * delta * delta * delta * (0.25 * qx * qx + 0.25 * qy * qy + 0.25 * qz * qz);
	process_unc[0][1] = q / 3.0 * delta * delta * delta * (-0.25 * qw * qx);
	process_unc[0][2] = q / 3.0 * delta * delta * delta * (-0.25 * qw * qy);
	process_unc[0][3] = q / 3.0 * delta * delta * delta * (-0.25 * qw * qz);
	process_unc[0][4] = q / 2.0 * delta * delta * (-0.5 * qx);
	process_unc[0][5] = q / 2.0 * delta * delta * (-0.5 * qy);
	process_unc[0][6] = q / 2.0 * delta * delta * (-0.5 * qz);
	process_unc[1][0] = q / 3.0 * delta * delta * delta * (-0.25 * qw * qx);
	process_unc[1][1] = q / 3.0 * delta * delta * delta * (0.25 * qw * qw + 0.25 * qy * qy + 0.25 * qz * qz);
	process_unc[1][2] = q / 3.0 * delta * delta * delta * (-0.25 * qx * qy);
	process_unc[1][3] = q / 3.0 * delta * delta * delta * (-0.25 * qx * qz);
	process_unc[1][4] = q / 2.0 * delta * delta * (0.5 * qw);
	process_unc[1][5] = q / 2.0 * delta * delta * (-0.5 * qz);
	process_unc[1][6] = q / 2.0 * delta * delta * (0.5 * qy);
	process_unc[2][0] = q / 3.0 * delta * delta * delta * (-0.25 * qw * qy);
	process_unc[2][1] = q / 3.0 * delta * delta * delta * (-0.25 * qx * qy);
	process_unc[2][2] = q / 3.0 * delta * delta * delta * (0.25 * qw * qw + 0.25 * qx * qx + 0.25 * qz * qz);
	process_unc[2][3] = q / 3.0 * delta * delta * delta * (-0.25 * qy * qz);
	process_unc[2][4] = q / 2.0 * delta * delta * (0.5 * qz);
	process_unc[2][5] = q / 2.0 * delta * delta * (0.5 * qw);
	process_unc[2][6] = q / 2.0 * delta * delta * (-0.5 * qx);
	process_unc[3][0] = q / 3.0 * delta * delta * delta * (-0.25 * qw * qz);
	process_unc[3][1] = q / 3.0 * delta * delta * delta * (-0.25 * qx * qz);
	process_unc[3][2] = q / 3.0 * delta * delta * delta * (-0.25 * qy * qz);
	process_unc[3][3] = q / 3.0 * delta * delta * delta * (0.25 * qw * qw + 0.25 * qx * qx + 0.25 * qy * qy);
	process_unc[3][4] = q / 2.0 * delta * delta * (-0.5 * qy);
	process_unc[3][5] = q / 2.0 * delta * delta * (0.5 * qx);
	process_unc[3][6] = q / 2.0 * delta * delta * (0.5 * qw);
	process_unc[4][0] = q / 2.0 * delta * delta * (-0.5 * qx);
	process_unc[4][1] = q / 2.0 * delta * delta * (0.5 * qw);
	process_unc[4][2] = q / 2.0 * delta * delta * (0.5 * qz);
	process_unc[4][3] = q / 2.0 * delta * delta * (-0.5 * qy);
	process_unc[4][4] = q * delta * (1);
	process_unc[4][5] = q * delta * (0);
	process_unc[4][6] = q * delta * (0);
	process_unc[5][0] = q / 2.0 * delta * delta * (-0.5 * qy);
	process_unc[5][1] = q / 2.0 * delta * delta * (-0.5 * qz);
	process_unc[5][2] = q / 2.0 * delta * delta * (0.5 * qw);
	process_unc[5][3] = q / 2.0 * delta * delta * (0.5 * qx);
	process_unc[5][4] = q * delta * (0);
	process_unc[5][5] = q * delta * (1);
	process_unc[5][6] = q * delta * (0);
	process_unc[6][0] = q / 2.0 * delta * delta * (-0.5 * qz);
	process_unc[6][1] = q / 2.0 * delta * delta * (0.5 * qy);
	process_unc[6][2] = q / 2.0 * delta * delta * (-0.5 * qx);
	process_unc[6][3] = q / 2.0 * delta * delta * (0.5 * qw);
	process_unc[6][4] = q * delta * (0);
	process_unc[6][5] = q * delta * (0);
	process_unc[6][6] = q * delta * (1);
}

void ConstantVelocityAccelGyroMagQuatMotionModel::operator()(
	double delta, vector<double>& state, vector<vector<double>>& jac, vector<vector<double>>& process_unc)
{
	derivs(delta, state, jac);
	getProcessUncertainty(delta, state, process_unc);
	predict(delta, state);
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::innovation(
	vector<double>& state, sensors::accel& accel, vector<double>& y)
{
	double qx, qy, qz, qw;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];

	// calculate innovation
	// y = z - h(x)
	y[0] = accel.x - (g * (-2 * qw * qy + 2 * qx * qz));
	y[1] = accel.y - (g * (2 * qw * qx + 2 * qy * qz));
	y[2] = accel.z - (g * (2 * qw * qw + 2 * qz * qz - 1));
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::innovation(
	vector<double>& state, sensors::gyro& gyro, vector<double>& y)
{
	// calculate innovation
	// y = z - h(x)
	y[0] = gyro.x - state[4];
	y[1] = gyro.y - state[5];
	y[2] = gyro.z - state[6];
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::innovation(
	vector<double>& state, sensors::mag& mag, vector<double>& y)
{
	double qx, qy, qz, qw;
	qw = state[0];
	qx = state[1];
	qy = state[2];
	qz = state[3];

	// calculate innovation
	// y = z - h(x)
	y[0] = mag.x - (mx * (2 * qw * qw + 2 * qx * qx - 1) + my * (2 * qw * qz + 2 * qx * qy) +
					mz * (-2 * qw * qy + 2 * qx * qz));
	y[1] = mag.y - (mx * (-2 * qw * qz + 2 * qx * qy) + my * (2 * qw * qw + 2 * qy * qy - 1) +
					mz * (2 * qw * qx + 2 * qy * qz));
	y[2] = mag.z - (mx * (2 * qw * qy + 2 * qx * qz) + my * (-2 * qw * qx + 2 * qy * qz) +
					mz * (2 * qw * qw + 2 * qz * qz - 1));
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::operator()(
	vector<double>& state,
	sensors::accel& accel,
	vector<double>& y,
	vector<vector<double>>& jac,
	vector<vector<double>>& measure_unc)
{
	innovation(state, accel, y);
	derivs(state, accel, jac);
	getMeasurementUncertainty(accel, measure_unc);
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::operator()(
	vector<double>& state,
	sensors::gyro& gyro,
	vector<double>& y,
	vector<vector<double>>& jac,
	vector<vector<double>>& measure_unc)
{
	innovation(state, gyro, y);
	derivs(state, gyro, jac);
	getMeasurementUncertainty(gyro, measure_unc);
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::operator()(
	vector<double>& state,
	sensors::mag& mag,
	vector<double>& y,
	vector<vector<double>>& jac,
	vector<vector<double>>& measure_unc)
{
	innovation(state, mag, y);
	derivs(state, mag, jac);
	getMeasurementUncertainty(mag, measure_unc);
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::derivs(
	vector<double>& state, sensors::accel& accel, vector<vector<double>>& jac)
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
	jac[2][1] = 0.0;

	jac[0][2] = -2 * g * qw;
	jac[1][2] = 2 * g * qz;
	jac[2][2] = 0.0;

	jac[0][3] = 2 * g * qx;
	jac[1][3] = 2 * g * qy;
	jac[2][3] = 4 * g * qz;

	jac[0][4] = 0.0;
	jac[1][4] = 0.0;
	jac[2][4] = 0.0;

	jac[0][5] = 0.0;
	jac[1][5] = 0.0;
	jac[2][5] = 0.0;

	jac[0][6] = 0.0;
	jac[1][6] = 0.0;
	jac[2][6] = 0.0;
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::derivs(
	vector<double>& state, sensors::gyro& gyro, vector<vector<double>>& jac)
{
	// H = dh/dx
	jac[0][0] = 0.0;
	jac[1][0] = 0.0;
	jac[2][0] = 0.0;

	jac[0][1] = 0.0;
	jac[1][1] = 0.0;
	jac[2][1] = 0.0;

	jac[0][2] = 0.0;
	jac[1][2] = 0.0;
	jac[2][2] = 0.0;

	jac[0][3] = 0.0;
	jac[1][3] = 0.0;
	jac[2][3] = 0.0;

	jac[0][4] = 1.0;
	jac[1][4] = 0.0;
	jac[2][4] = 0.0;

	jac[0][5] = 0.0;
	jac[1][5] = 1.0;
	jac[2][5] = 0.0;

	jac[0][6] = 0.0;
	jac[1][6] = 0.0;
	jac[2][6] = 1.0;
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::derivs(
	vector<double>& state, sensors::mag& mag, vector<vector<double>>& jac)
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

	jac[0][4] = 0.0;
	jac[1][4] = 0.0;
	jac[2][4] = 0.0;

	jac[0][5] = 0.0;
	jac[1][5] = 0.0;
	jac[2][5] = 0.0;

	jac[0][6] = 0.0;
	jac[1][6] = 0.0;
	jac[2][6] = 0.0;
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::getMeasurementUncertainty(
	sensors::accel& accel, vector<vector<double>>& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = accel.xunc;
	measure_unc[1][1] = accel.yunc;
	measure_unc[2][2] = accel.zunc;
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::getMeasurementUncertainty(
	sensors::gyro& gyro, vector<vector<double>>& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = gyro.xunc;
	measure_unc[1][1] = gyro.yunc;
	measure_unc[2][2] = gyro.zunc;
}

void ConstantVelocityAccelGyroMagQuatMeasurementModel::getMeasurementUncertainty(
	sensors::mag& mag, vector<vector<double>>& measure_unc)
{
	// fill measurement uncertainty matrix
	measure_unc[0][0] = mag.xunc;
	measure_unc[1][1] = mag.yunc;
	measure_unc[2][2] = mag.zunc;
}