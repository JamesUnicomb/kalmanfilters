#ifndef _SENSORS_HPP_
#define _SENSORS_HPP_

namespace sensors
{
struct base
{
	base(double x, double y, double z);
	base(double x, double y, double z, double xunc, double yunc, double zunc);
	double x, y, z;
	double xunc, yunc, zunc;
};

struct accel : base
{
	accel(double x, double y, double z);
	accel(double x, double y, double z, double xunc, double yunc, double zunc);
};

struct gyro : base
{
	gyro(double x, double y, double z);
	gyro(double x, double y, double z, double xunc, double yunc, double zunc);
};

struct mag : base
{
	mag(double x, double y, double z);
	mag(double x, double y, double z, double xunc, double yunc, double zunc);
};
}; // namespace sensors

#endif