#ifndef _TRACKING_HPP_
#define _TRACKING_HPP_

#include "nr3.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"
#include <vector>
#include <cmath>

class Tracking
{
public:
	double process_unc; // noise constants
	std::vector<double> state;
	std::vector<std::vector<double>> state_unc;

	std::vector<double> innovation;
	std::vector<std::vector<double>> innovation_unc;
	std::vector<std::vector<double>> measure_unc;

	std::vector<std::vector<double>> gain;

	Tracking(double process_unc);

	void update(const sensors::accel& accel, double unc);
	void update(const sensors::gyro& gyro, double unc);
	void update(const sensors::mag& mag, double unc);
	void update();
	void predict(double dt);

	std::vector<std::vector<double>> jac, jacT;
	std::vector<std::vector<double>> innovation_unc_inv;
	std::vector<double> dx;
	std::vector<std::vector<double>> eye, tmp, tmpunc;

private:
	const double g = 9.81;
	const int statedim = 6;
	const int measuredim = 3;
};

#endif /* _TRACKING_HPP_ */