#ifndef _CVACCLGYROMODEL_HPP_
#define _CVACCLGYROMODEL_HPP_
#include <vector>
#include "sensors/sensors.hpp"

struct ConstantVelocityAccelGyroMotionModel
{
	void operator()(
		double delta,
		std::vector<double>& state,
		std::vector<std::vector<double>>& jac,
		std::vector<std::vector<double>>& process_unc);

	void predict(double delta, std::vector<double>& state);
	void derivs(double delta, std::vector<double>& state, std::vector<std::vector<double>>& jac);
	void getProcessUncertainty(double delta, std::vector<std::vector<double>>& process_unc);

	const int statedim = 5;
	double q;
};

struct ConstantVelocityAccelGyroMeasurementModel
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

	void innovation(std::vector<double>& state, sensors::accel& accel, std::vector<double>& y);
	void innovation(std::vector<double>& state, sensors::gyro& gyro, std::vector<double>& y);
	void derivs(std::vector<double>& state, sensors::accel& accel, std::vector<std::vector<double>>& jac);
	void derivs(std::vector<double>& state, sensors::gyro& gyro, std::vector<std::vector<double>>& jac);
	void getMeasurementUncertainty(sensors::accel& accel, std::vector<std::vector<double>>& measure_unc);
	void getMeasurementUncertainty(sensors::gyro& gyro, std::vector<std::vector<double>>& measure_unc);
	void final(std::vector<double>& state, std::vector<std::vector<double>>& state_unc) { }

	const int statedim = 5;
	const int measuredim = 3;

	// earth gravity
	const double g = 9.81;
};

#endif