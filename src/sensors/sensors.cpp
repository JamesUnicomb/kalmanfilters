#include "sensors.hpp"
#include "linalg/linalg.hpp"

using namespace linalg;

sensors::base::base(double x, double y, double z)
	: x(x)
	, y(y)
	, z(z)
	, xunc(0.0)
	, yunc(0.0)
	, zunc(0.0)
	, v(3)
{
	// initialise measurement vector
	v[0] = x;
	v[1] = y;
	v[2] = z;
}
sensors::base::base(double x, double y, double z, double xunc, double yunc, double zunc)
	: x(x)
	, y(y)
	, z(z)
	, xunc(xunc)
	, yunc(yunc)
	, zunc(zunc)
	, v(3)
	, S(3, 3)
{
	// initialise measurement vector
	v[0] = x;
	v[1] = y;
	v[2] = z;

	// initialise uncertainty matrix
	S[0][0] = xunc;
	S[0][1] = 0.0;
	S[0][2] = 0.0;

	S[1][0] = 0.0;
	S[1][1] = yunc;
	S[1][2] = 0.0;

	S[2][0] = 0.0;
	S[2][1] = 0.0;
	S[2][2] = zunc;
}

Vector& sensors::base::vec()
{
	return v;
}

Matrix& sensors::base::unc()
{
	return S;
}

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