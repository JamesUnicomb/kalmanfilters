#ifndef _KALMANFILTERS_HPP_
#define _KALMANFILTERS_HPP_

#include "nr3.hpp"
#include "linalg/linalg.hpp"
#include "quaternion/quaternion.hpp"
#include "sensors/sensors.hpp"
#include "igrf/igrf.hpp"
#include "kalmanfilter/ExtendedKalmanFilter.hpp"
#include "kalmanfilter/UnscentedKalmanFilter.hpp"
#include "models/ConstantPositionAccelMagQuat.hpp"
#include "models/ConstantVelocityAccelGyroMagQuat.hpp"

class GeomagModel
{
public:
	GeomagModel()
		: igrf(2023.0)
	{
		igrf.get_field(lon_, lat_, alt_, mx_, my_, mz_);
	}
	void operator()(double& mx, double& my, double& mz)
	{
		mx = mx_;
		my = my_;
		mz = mz_;
	}
	void setParameters(double lon, double lat, double alt)
	{
		lon_ = lon;
		lat_ = lat;
		alt_ = alt;
		igrf.get_field(lon_, lat_, alt_, mx_, my_, mz_);
	}

private:
	IGRF igrf;
	double lon_, lat_, alt_;
	double mx_, my_, mz_;
};

typedef ExtendedKalmanFilter<
	ConstantPositionAccelMagQuatMotionModel,
	ConstantPositionAccelMagQuatMeasurementModel<GeomagModel>>
	cpqekf;

typedef UnscentedKalmanFilter<
	ConstantPositionAccelMagQuatMotionModel,
	ConstantPositionAccelMagQuatMeasurementModel<GeomagModel>>
	cpqukf;

typedef ExtendedKalmanFilter<
	ConstantVelocityAccelGyroMagQuatMotionModel,
	ConstantVelocityAccelGyroMagQuatMeasurementModel<GeomagModel>>
	cvqekf;

typedef UnscentedKalmanFilter<
	ConstantVelocityAccelGyroMagQuatMotionModel,
	ConstantVelocityAccelGyroMagQuatMeasurementModel<GeomagModel>>
	cvqukf;

#endif