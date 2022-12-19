#ifndef _CSEKF_HPP_
#define _CSEKF_HPP_

#include "nr3.hpp"
#include <vector>
#include <cmath>

class ConstantStateExtendedKalmanFilter
{
public:
	double process_unc, measurement_unc; // noise constants
	std::vector<double> state;
	std::vector<std::vector<double>> state_unc;

	std::vector<double> innovation;
    std::vector<std::vector<double>> innovation_unc;
	
    std::vector<std::vector<double>> gain;
	

	ConstantStateExtendedKalmanFilter(double process_unc, double measurement_unc)
		: process_unc(process_unc)
		, measurement_unc(measurement_unc)
	{
		// state is 2x1 and sigma is 2x2
		state = std::vector<double>(statedim, 0.0);
		state_unc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
        state_unc[0][0] = state_unc[1][1] = 1.0;

		// innovation is 3x1
		innovation = std::vector<double>(measuredim, 0.0);
        innovation_unc = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));
        innovation_unc_inv = std::vector<std::vector<double>>(measuredim, std::vector<double>(measuredim, 0.0));

        // fill tmp matrices
        dh = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));
        dhS = std::vector<std::vector<double>>(measuredim, std::vector<double>(statedim, 0.0));
        dhT = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));
        dhTSinv = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

        // kalman gain is 2x3
        gain = std::vector<std::vector<double>>(statedim, std::vector<double>(measuredim, 0.0));

        // update variables
        dx = std::vector<double>(statedim, 0.0);
        eye = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
        eye[0][0] = eye[1][1] = 1.0;

        khtmp = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
        tmpunc = std::vector<std::vector<double>>(statedim, std::vector<double>(statedim, 0.0));
	}

	void update(const std::vector<double>& accel);
	void predict(double dt);

    std::vector<std::vector<double>> dh, dhS, dhT, dhTSinv;
    std::vector<std::vector<double>> innovation_unc_inv;
    std::vector<double> dx;
    std::vector<std::vector<double>> eye, khtmp, tmpunc;
private:
    const double g = 9.81;
    const int statedim = 2;
    const int measuredim = 3;
    // temp matrices for calculations
};

#endif /* _CSEKF_HPP_ */