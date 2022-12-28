#ifndef _EKF_HPP_
#define _EKF_HPP_

#include "nr3.hpp"
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"
#include <vector>
#include <cmath>

struct MotionModel
{
	virtual int getStateDim() const = 0;
	virtual void predict(
		double delta, std::vector<double>& state, std::vector<std::vector<double>>& process_unc) const = 0;
	virtual void
	derivs(double delta, std::vector<double>& state, std::vector<std::vector<double>>& jacobian) const = 0;
};

template <typename Z>
struct MeasurementModel
{
	virtual int getMeasurementDim() const = 0;
	virtual void update(
		const Z& z, std::vector<double>& state, std::vector<std::vector<double>>& measurement_unc) const = 0;
	virtual void
	derivs(const Z& z, std::vector<double>& state, std::vector<std::vector<double>>& jacobian) const = 0;
};

template <typename F, typename H, typename Z>
class ExtendedKalmanFilter
{
public:
	/*
    x = f(x,dt)
    sigma = df/dx * sigma * df/dx^T + Q

    y = z - h(x)
    S = dh/dx * sigma * dh/dx^T + R
    K = dh/dx * sigma * S^-1

    x = x + K * y
    sigma = (I - K * dh/dx) * sigma
    */
    
	std::vector<double> state;
	std::vector<std::vector<double>> state_unc;

    // Q and R
    std::vector<std::vector<double>> process_unc, measure_unc;

    // y and S
	std::vector<double> innovation;
	std::vector<std::vector<double>> innovation_unc;

    // K
	std::vector<std::vector<double>> gain;

	ExtendedKalmanFilter(int statedim, int measuredim);

	void predict(double delta, F f);
	void update(H h, const Z& z);

private:
	const double g = 9.81;
	const int statedim;
	const int measuredim;

    // temp matrices for calculations
	std::vector<std::vector<double>> fjac, fjacT, hjac, hjacT;
	std::vector<std::vector<double>> innovation_unc_inv;
	std::vector<double> dx;
	std::vector<std::vector<double>> eye, tmp, tmpunc;
};

#endif /* _EKF_HPP_ */