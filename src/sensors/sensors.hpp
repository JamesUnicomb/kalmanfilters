#ifndef _SENSORS_HPP_
#define _SENSORS_HPP_

#include "linalg/linalg.hpp"

namespace sensors
{
class base
{
public:
	const double x, y, z;
	const double xunc, yunc, zunc;
	linalg::Vector v;
	linalg::Matrix S;
	base(double x, double y, double z);
	base(double x, double y, double z, double xunc, double yunc, double zunc);
	linalg::Vector& vec();
	linalg::Matrix& unc();
};

class accel : public base
{
public:
	accel(double x, double y, double z);
	accel(double x, double y, double z, double xunc, double yunc, double zunc);
};

class gyro : public base
{
public:
	gyro(double x, double y, double z);
	gyro(double x, double y, double z, double xunc, double yunc, double zunc);
};

class mag : public base
{
public:
	mag(double x, double y, double z);
	mag(double x, double y, double z, double xunc, double yunc, double zunc);
};
}; // namespace sensors

#endif