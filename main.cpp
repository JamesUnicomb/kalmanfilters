#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

#include "nr3.hpp"
#include "sensors/sensors.hpp"
#include "kalmanfilter/ExtendedKalmanFilter.hpp"
#include "models/ConstantPositionAccel.hpp"
#include "models/ConstantVelocityAccelGyro.hpp"
#include "models/ConstantVelocityAccelGyroMag.hpp"

// clang-format off
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

using namespace std;
namespace py = pybind11;

PYBIND11_MODULE(kalmanfilters, mod) {
    mod.doc() = "";

    py::module sensors = mod.def_submodule("sensors", "sensors to use as input into Kalman filters.");
    py::class_<sensors::accel>(sensors, "accel")
        .def(py::init<double, double, double>())
        .def(py::init<double, double, double, double, double, double>())
        .def_readwrite("x", &sensors::accel::x)
        .def_readwrite("y", &sensors::accel::y)
        .def_readwrite("z", &sensors::accel::z)
        .def_readwrite("xunc", &sensors::accel::xunc)
        .def_readwrite("yunc", &sensors::accel::yunc)
        .def_readwrite("zunc", &sensors::accel::zunc);
    py::class_<sensors::gyro>(sensors, "gyro")
        .def(py::init<double, double, double>())
        .def(py::init<double, double, double, double, double, double>())
        .def_readwrite("x", &sensors::accel::x)
        .def_readwrite("y", &sensors::accel::y)
        .def_readwrite("z", &sensors::accel::z)
        .def_readwrite("xunc", &sensors::accel::xunc)
        .def_readwrite("yunc", &sensors::accel::yunc)
        .def_readwrite("zunc", &sensors::accel::zunc);
    py::class_<sensors::mag>(sensors, "mag")
        .def(py::init<double, double, double>())
        .def(py::init<double, double, double, double, double, double>())
        .def_readwrite("x", &sensors::accel::x)
        .def_readwrite("y", &sensors::accel::y)
        .def_readwrite("z", &sensors::accel::z)
        .def_readwrite("xunc", &sensors::accel::xunc)
        .def_readwrite("yunc", &sensors::accel::yunc)
        .def_readwrite("zunc", &sensors::accel::zunc);

    typedef ExtendedKalmanFilter<ConstantPositionAccelMotionModel, ConstantPositionAccelMeasurementModel> cpekf;
    py::class_<cpekf>(mod, "cpekf")
        .def(py::init<double>())
        .def("predict", &cpekf::predict)
        .def("update", &cpekf::update<sensors::accel&>)
        .def_readwrite("state", &cpekf::state)
        .def_readwrite("state_unc", &cpekf::state_unc)
        .def_readwrite("innovation", &cpekf::innovation)
        .def_readwrite("innovation_unc", &cpekf::innovation_unc)
        .def_readwrite("dhdx", &cpekf::dhdx);

    typedef ExtendedKalmanFilter<ConstantVelocityAccelGyroMagMotionModel, ConstantVelocityAccelGyroMagMeasurementModel> cvekf;
    py::class_<cvekf>(mod, "cvekf")
        .def(py::init<double>())
        .def("predict", &cvekf::predict)
        .def("update", &cvekf::update<sensors::accel&>)
        .def("update", &cvekf::update<sensors::gyro&>)
        .def("update", &cvekf::update<sensors::mag&>)
        .def_readwrite("state", &cvekf::state)
        .def_readwrite("state_unc", &cvekf::state_unc)
        .def_readwrite("innovation", &cvekf::innovation)
        .def_readwrite("innovation_unc", &cvekf::innovation_unc)
        .def_readwrite("dhdx", &cvekf::dhdx);


#ifdef VERSION_INFO
    mod.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    mod.attr("__version__") = "dev";
#endif
}
// clang-format on