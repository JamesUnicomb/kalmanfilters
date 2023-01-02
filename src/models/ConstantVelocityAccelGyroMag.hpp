#ifndef _CVACCLGYROMAGMODEL_HPP_
#define _CVACCLGYROMAGMODEL_HPP_
#include <vector>
#include "sensors/sensors.hpp"

struct ConstantVelocityAccelGyroMagMotionModel
{
	void operator()(
		double delta,
		std::vector<double>& state,
		std::vector<std::vector<double>>& jac,
		std::vector<std::vector<double>>& process_unc);

	void predict(double delta, std::vector<double>& state);
	void derivs(double delta, std::vector<double>& state, std::vector<std::vector<double>>& jac);
	void getProcessUncertainty(double delta, std::vector<std::vector<double>>& process_unc);

	const int statedim = 6;
	double q;
};

struct ConstantVelocityAccelGyroMagMeasurementModel
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

	void predict(std::vector<double>& state, sensors::accel& accel, std::vector<double>& h);
	void predict(std::vector<double>& state, sensors::gyro& gyro, std::vector<double>& h);
	void predict(std::vector<double>& state, sensors::mag& mag, std::vector<double>& h);
	void innovation(std::vector<double>& state, sensors::accel& accel, std::vector<double>& y);
	void innovation(std::vector<double>& state, sensors::gyro& gyro, std::vector<double>& y);
	void innovation(std::vector<double>& state, sensors::mag& mag, std::vector<double>& y);
	void derivs(std::vector<double>& state, sensors::accel& accel, std::vector<std::vector<double>>& jac);
	void derivs(std::vector<double>& state, sensors::gyro& gyro, std::vector<std::vector<double>>& jac);
	void derivs(std::vector<double>& state, sensors::mag& mag, std::vector<std::vector<double>>& jac);
	void getMeasurementUncertainty(sensors::accel& accel, std::vector<std::vector<double>>& measure_unc);
	void getMeasurementUncertainty(sensors::gyro& gyro, std::vector<std::vector<double>>& measure_unc);
	void getMeasurementUncertainty(sensors::mag& mag, std::vector<std::vector<double>>& measure_unc);

	const int statedim = 6;
	const int measuredim = 3;

	// earth gravity
	const double g = 9.81;

	// static earth magnetic field vector in sydney
	const double mx = 24.0475, my = 5.4344, mz = 51.4601;
};

#endif