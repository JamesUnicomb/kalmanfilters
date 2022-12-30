#ifndef _CPACCLMODEL_HPP_
#define _CPACCLMODEL_HPP_
#include <vector>
#include "sensors/sensors.hpp"

struct ConstantPositionAccelMotionModel
{
	void predict(double delta, std::vector<double>& state);
	void derivs(double delta, std::vector<double>& state, std::vector<std::vector<double>>& jac);
	void getProcessUncertainty(double delta, std::vector<std::vector<double>>& process_unc);
	void operator()(double delta, std::vector<double>& state);
	const int statedim = 2;
	double q;
};

struct ConstantPositionAccelMeasurementModel
{
	void update(std::vector<double>& state, sensors::accel& accel, std::vector<double>& innovation);

	void operator()(std::vector<double>& state, sensors::accel& accel, std::vector<double>& innovation);

	void derivs(std::vector<double>& state, sensors::accel& accel, std::vector<std::vector<double>>& jac);

	void getMeasurementUncertainty(sensors::accel& accel, std::vector<std::vector<double>>& measure_unc);

	const int statedim = 2;
	const int measuredim = 3;

	// earth gravity
	const double g = 9.81;
};

#endif