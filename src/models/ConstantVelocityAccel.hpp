#ifndef _CVACCLMODEL_HPP_
#define _CVACCLMODEL_HPP_
#include <vector>
#include "sensors/sensors.hpp"

struct ConstantVelocityAccelMotionModel
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

struct ConstantVelocityAccelMeasurementModel
{
	void operator()(
		std::vector<double>& state,
		sensors::accel& accel,
		std::vector<double>& y,
		std::vector<std::vector<double>>& jac,
		std::vector<std::vector<double>>& measure_unc);

	void innovation(std::vector<double>& state, sensors::accel& accel, std::vector<double>& y);
	void derivs(std::vector<double>& state, sensors::accel& accel, std::vector<std::vector<double>>& jac);
	void getMeasurementUncertainty(sensors::accel& accel, std::vector<std::vector<double>>& measure_unc);

	const int statedim = 4;
	const int measuredim = 3;

	// earth gravity
	const double g = 9.81;
};

#endif