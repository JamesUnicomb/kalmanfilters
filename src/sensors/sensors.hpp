#ifndef _SENSORS_HPP_
#define _SENSORS_HPP_

namespace sensors
{
struct accel
{
	accel(double x, double y, double z);
	double x, y, z;
};

struct gyro
{
	gyro(double x, double y, double z);
	double x, y, z;
};

struct mag
{
	mag(double x, double y, double z);
	double x, y, z;
};
}; // namespace sensors

#endif