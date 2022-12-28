#ifndef _SENSORS_HPP_
#define _SENSORS_HPP_

namespace sensors
{
struct base {
	base(double x, double y, double z);
    double x, y, z;
};

struct accel : base
{
	accel(double x, double y, double z);
};

struct gyro : base
{
	gyro(double x, double y, double z);
};

struct mag : base
{
	mag(double x, double y, double z);
};
}; // namespace sensors

#endif