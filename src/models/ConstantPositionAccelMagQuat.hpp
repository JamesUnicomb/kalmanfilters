#ifndef _CPACCLMAGQUATMODEL_HPP_
#define _CPACCLMAGQUATMODEL_HPP_
#include <vector>
#include "linalg/linalg.hpp"
#include "sensors/sensors.hpp"

struct ConstantPositionAccelMagQuatMotionModel
{
	void operator()(double delta, linalg::Vector& state, linalg::Matrix& jac, linalg::Matrix& process_unc);

	void predict(double delta, linalg::Vector& state);
	void derivs(double delta, linalg::Vector& state, linalg::Matrix& jac);
	void getProcessUncertainty(double delta, linalg::Vector& state, linalg::Matrix& process_unc);
	void setParameters(double q)
	{
		q_ = q;
	}
	void final(linalg::Vector& state, linalg::Matrix& state_unc) { }

	const int statedim = 4;
	double q_;
};

template <typename M>
struct ConstantPositionAccelMagQuatMeasurementModel
{
	void operator()(linalg::Vector& state, sensors::accel& accel, linalg::Vector& y, linalg::Matrix& jac);
	void operator()(linalg::Vector& state, sensors::mag& mag, linalg::Vector& y, linalg::Matrix& jac);

	void predict(linalg::Vector& state, sensors::accel& accel, linalg::Vector& h);
	void predict(linalg::Vector& state, sensors::mag& mag, linalg::Vector& h);
	void innovation(linalg::Vector& state, sensors::accel& accel, linalg::Vector& y);
	void innovation(linalg::Vector& state, sensors::mag& mag, linalg::Vector& y);
	void derivs(linalg::Vector& state, sensors::accel& accel, linalg::Matrix& jac);
	void derivs(linalg::Vector& state, sensors::mag& mag, linalg::Matrix& jac);
	void setParameters(double lon, double lat, double alt)
	{
		m.setParameters(lon, lat, alt);
		// get magnetic field vector values
		m(mx, my, mz);
	}
	void final(linalg::Vector& state, linalg::Matrix& state_unc)
	{
		// normalize quaternion
		double qn = 0.0;
		double qw, qx, qy, qz;
		qw = state[0];
		qx = state[1];
		qy = state[2];
		qz = state[3];
		qn = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		state[0] = qw / qn;
		state[1] = qx / qn;
		state[2] = qy / qn;
		state[3] = qz / qn;
	}

	const int statedim = 4;
	const int measuredim = 3;

	// magnetic field model
	M m;

	// earth magnetic field vector
	double mx, my, mz;

	// earth gravity
	const double g = 9.81;
};

#include "ConstantPositionAccelMagQuat.cpp"

#endif