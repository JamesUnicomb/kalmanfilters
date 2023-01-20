#ifndef _CPACCLMODEL_HPP_
#define _CPACCLMODEL_HPP_
#include <vector>
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

struct ConstantPositionAccelMotionModel
{
	void operator()(double delta, linalg::Vector& state, linalg::Matrix& jac, linalg::Matrix& process_unc);

	void predict(double delta, linalg::Vector& state);
	void derivs(double delta, linalg::Vector& state, linalg::Matrix& jac);
	void getProcessUncertainty(double delta, linalg::Matrix& process_unc);

	const int statedim = 2;
	double q;
};

struct ConstantPositionAccelMeasurementModel
{
	void operator()(
		linalg::Vector& state,
		sensors::accel& accel,
		linalg::Vector& y,
		linalg::Matrix& jac,
		linalg::Matrix& measure_unc);

	void innovation(linalg::Vector& state, sensors::accel& accel, linalg::Vector& y);
	void derivs(linalg::Vector& state, sensors::accel& accel, linalg::Matrix& jac);
	void getMeasurementUncertainty(sensors::accel& accel, linalg::Matrix& measure_unc);
	void final(linalg::Vector& state, linalg::Matrix& state_unc) { }

	const int statedim = 2;
	const int measuredim = 3;

	// earth gravity
	const double g = 9.81;
};

#endif