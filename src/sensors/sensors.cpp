#include "sensors.hpp"

sensors::base::base(double x, double y, double z) : x(x), y(y), z(z) {}

sensors::accel::accel(double x, double y, double z) : base(x,y,z) {}

sensors::gyro::gyro(double x, double y, double z) : base(x,y,z) {}

sensors::mag::mag(double x, double y, double z) : base(x,y,z) {}