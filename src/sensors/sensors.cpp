#include "sensors.hpp"

sensors::base::base(double x, double y, double z)
	: x(x)
	, y(y)
	, z(z)
{ }
sensors::base::base(double x, double y, double z, double xunc, double yunc, double zunc)
	: x(x)
	, y(y)
	, z(z)
	, xunc(xunc)
	, yunc(yunc)
	, zunc(zunc)
{ }

sensors::accel::accel(double x, double y, double z)
	: base(x, y, z)
{ }
sensors::accel::accel(double x, double y, double z, double xunc, double yunc, double zunc)
	: base(x, y, z, xunc, yunc, zunc)
{ }

sensors::gyro::gyro(double x, double y, double z)
	: base(x, y, z)
{ }
sensors::gyro::gyro(double x, double y, double z, double xunc, double yunc, double zunc)
	: base(x, y, z, xunc, yunc, zunc)
{ }

sensors::mag::mag(double x, double y, double z)
	: base(x, y, z)
{ }
sensors::mag::mag(double x, double y, double z, double xunc, double yunc, double zunc)
	: base(x, y, z, xunc, yunc, zunc)
{ }