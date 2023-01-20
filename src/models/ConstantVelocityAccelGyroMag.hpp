#ifndef _CVACCLGYROMAGMODEL_HPP_
#define _CVACCLGYROMAGMODEL_HPP_
#include <vector>
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

struct ConstantVelocityAccelGyroMagMotionModel
{
	void operator()(double delta, linalg::Vector& state, linalg::Matrix& jac, linalg::Matrix& process_unc);

	void predict(double delta, linalg::Vector& state);
	void derivs(double delta, linalg::Vector& state, linalg::Matrix& jac);
	void getProcessUncertainty(double delta, linalg::Matrix& process_unc);
	void final(linalg::Vector& state, linalg::Matrix& state_unc) { }

	const int statedim = 6;
	double q;
};

struct ConstantVelocityAccelGyroMagMeasurementModel
{
	void operator()(
		linalg::Vector& state,
		sensors::accel& accel,
		linalg::Vector& y,
		linalg::Matrix& jac,
		linalg::Matrix& measure_unc);
	void operator()(
		linalg::Vector& state,
		sensors::gyro& gyro,
		linalg::Vector& y,
		linalg::Matrix& jac,
		linalg::Matrix& measure_unc);
	void operator()(
		linalg::Vector& state,
		sensors::mag& mag,
		linalg::Vector& y,
		linalg::Matrix& jac,
		linalg::Matrix& measure_unc);

	void predict(linalg::Vector& state, sensors::accel& accel, linalg::Vector& h);
	void predict(linalg::Vector& state, sensors::gyro& gyro, linalg::Vector& h);
	void predict(linalg::Vector& state, sensors::mag& mag, linalg::Vector& h);
	void innovation(linalg::Vector& state, sensors::accel& accel, linalg::Vector& y);
	void innovation(linalg::Vector& state, sensors::gyro& gyro, linalg::Vector& y);
	void innovation(linalg::Vector& state, sensors::mag& mag, linalg::Vector& y);
	void derivs(linalg::Vector& state, sensors::accel& accel, linalg::Matrix& jac);
	void derivs(linalg::Vector& state, sensors::gyro& gyro, linalg::Matrix& jac);
	void derivs(linalg::Vector& state, sensors::mag& mag, linalg::Matrix& jac);
	void getMeasurementUncertainty(sensors::accel& accel, linalg::Matrix& measure_unc);
	void getMeasurementUncertainty(sensors::gyro& gyro, linalg::Matrix& measure_unc);
	void getMeasurementUncertainty(sensors::mag& mag, linalg::Matrix& measure_unc);
	void final(linalg::Vector& state, linalg::Matrix& state_unc) { }

	const int statedim = 6;
	const int measuredim = 3;

	// earth gravity
	const double g = 9.81;

	// static earth magnetic field vector in sydney
	const double mx = 24.0475, my = 5.4344, mz = 51.4601;
};

#endif