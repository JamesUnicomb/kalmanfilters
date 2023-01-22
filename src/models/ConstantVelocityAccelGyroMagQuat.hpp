#ifndef _CVACCLGYROMAGQUATMODEL_HPP_
#define _CVACCLGYROMAGQUATMODEL_HPP_
#include <vector>
#include <cmath>
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

struct ConstantVelocityAccelGyroMagQuatMotionModel
{
	void operator()(double delta, linalg::Vector& state, linalg::Matrix& jac, linalg::Matrix& process_unc);

	void predict(double delta, linalg::Vector& state);
	void derivs(double delta, linalg::Vector& state, linalg::Matrix& jac);
	void getProcessUncertainty(double delta, linalg::Vector& state, linalg::Matrix& process_unc);
	void final(linalg::Vector& state, linalg::Matrix& state_unc)
	{
		// normalize quaternion
		double qn = 0.0;
		double qw, qx, qy, qz;
		qw = state[0];
		qx = state[1];
		qy = state[2];
		qz = state[3];
		qn = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		state[0] = qw / qn;
		state[1] = qx / qn;
		state[2] = qy / qn;
		state[3] = qz / qn;
	}

	const int statedim = 7;
	double q;
};

struct ConstantVelocityAccelGyroMagQuatMeasurementModel
{
	void operator()(linalg::Vector& state, sensors::accel& accel, linalg::Vector& y, linalg::Matrix& jac);
	void operator()(linalg::Vector& state, sensors::gyro& gyro, linalg::Vector& y, linalg::Matrix& jac);
	void operator()(linalg::Vector& state, sensors::mag& mag, linalg::Vector& y, linalg::Matrix& jac);

	void predict(linalg::Vector& state, sensors::accel& accel, linalg::Vector& h);
	void predict(linalg::Vector& state, sensors::gyro& gyro, linalg::Vector& h);
	void predict(linalg::Vector& state, sensors::mag& mag, linalg::Vector& h);
	void innovation(linalg::Vector& state, sensors::accel& accel, linalg::Vector& y);
	void innovation(linalg::Vector& state, sensors::gyro& gyro, linalg::Vector& y);
	void innovation(linalg::Vector& state, sensors::mag& mag, linalg::Vector& y);
	void derivs(linalg::Vector& state, sensors::accel& accel, linalg::Matrix& jac);
	void derivs(linalg::Vector& state, sensors::gyro& gyro, linalg::Matrix& jac);
	void derivs(linalg::Vector& state, sensors::mag& mag, linalg::Matrix& jac);
	void final(linalg::Vector& state, linalg::Matrix& state_unc)
	{
		// normalize quaternion
		double qn = 0.0;
		double qw, qx, qy, qz;
		qw = state[0];
		qx = state[1];
		qy = state[2];
		qz = state[3];
		qn = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		state[0] = qw / qn;
		state[1] = qx / qn;
		state[2] = qy / qn;
		state[3] = qz / qn;
	}

	const int statedim = 7;
	const int measuredim = 3;

	// earth gravity
	const double g = 9.81;

	// static earth magnetic field vector in sydney
	const double mx = 24.0475, my = 5.4344, mz = 51.4601;
};

#endif