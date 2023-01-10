#ifndef _CVACCLGYROMAGQUATMODEL_HPP_
#define _CVACCLGYROMAGQUATMODEL_HPP_
#include <vector>
#include <cmath>
#include "sensors/sensors.hpp"

struct ConstantVelocityAccelGyroMagQuatMotionModel
{
	void operator()(
		double delta,
		std::vector<double>& state,
		std::vector<std::vector<double>>& jac,
		std::vector<std::vector<double>>& process_unc);

	void predict(double delta, std::vector<double>& state);
	void derivs(double delta, std::vector<double>& state, std::vector<std::vector<double>>& jac);
	void getProcessUncertainty(
		double delta, std::vector<double>& state, std::vector<std::vector<double>>& process_unc);

	const int statedim = 7;
	double q;
};

struct ConstantVelocityAccelGyroMagQuatMeasurementModel
{
	void operator()(
		std::vector<double>& state,
		sensors::accel& accel,
		std::vector<double>& y,
		std::vector<std::vector<double>>& jac,
		std::vector<std::vector<double>>& measure_unc);
	void operator()(
		std::vector<double>& state,
		sensors::gyro& gyro,
		std::vector<double>& y,
		std::vector<std::vector<double>>& jac,
		std::vector<std::vector<double>>& measure_unc);
	void operator()(
		std::vector<double>& state,
		sensors::mag& mag,
		std::vector<double>& y,
		std::vector<std::vector<double>>& jac,
		std::vector<std::vector<double>>& measure_unc);

	void innovation(std::vector<double>& state, sensors::accel& accel, std::vector<double>& y);
	void innovation(std::vector<double>& state, sensors::gyro& gyro, std::vector<double>& y);
	void innovation(std::vector<double>& state, sensors::mag& mag, std::vector<double>& y);
	void derivs(std::vector<double>& state, sensors::accel& accel, std::vector<std::vector<double>>& jac);
	void derivs(std::vector<double>& state, sensors::gyro& gyro, std::vector<std::vector<double>>& jac);
	void derivs(std::vector<double>& state, sensors::mag& mag, std::vector<std::vector<double>>& jac);
	void getMeasurementUncertainty(sensors::accel& accel, std::vector<std::vector<double>>& measure_unc);
	void getMeasurementUncertainty(sensors::gyro& gyro, std::vector<std::vector<double>>& measure_unc);
	void getMeasurementUncertainty(sensors::mag& mag, std::vector<std::vector<double>>& measure_unc);
	void final(std::vector<double>& state, std::vector<std::vector<double>>& state_unc)
	{
		// normalize quaternion
		double qn = 0;
		double qw, qx, qy, qz;
		qw = state[0];
		qx = state[1];
		qy = state[2];
		qz = state[3];
		qn = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		state[0] *= 1.0 / qn;
		state[1] *= 1.0 / qn;
		state[2] *= 1.0 / qn;
		state[3] *= 1.0 / qn;
	}

	const int statedim = 7;
	const int measuredim = 3;

	// earth gravity
	const double g = 9.81;

	// static earth magnetic field vector in sydney
	const double mx = 24.0475, my = 5.4344, mz = 51.4601;
};

#endif