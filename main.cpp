#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

#include "nr3.hpp"
#include "sensors/sensors.hpp"
#include "extended_kalman_filter/ConstantStateExtendedKalmanFilter.hpp"

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
        .def_readwrite("x", &sensors::accel::x)
        .def_readwrite("y", &sensors::accel::y)
        .def_readwrite("z", &sensors::accel::z);
    py::class_<sensors::gyro>(sensors, "gyro")
        .def(py::init<double, double, double>())
        .def_readwrite("x", &sensors::gyro::x)
        .def_readwrite("y", &sensors::gyro::y)
        .def_readwrite("z", &sensors::gyro::z);
    py::class_<sensors::mag>(sensors, "mag")
        .def(py::init<double, double, double>())
        .def_readwrite("x", &sensors::mag::x)
        .def_readwrite("y", &sensors::mag::y)
        .def_readwrite("z", &sensors::mag::z);;

    py::class_<ConstantStateExtendedKalmanFilter>(mod, "ConstantStateExtendedKalmanFilter")
        .def(py::init<double, double>())
        .def("predict", &ConstantStateExtendedKalmanFilter::predict)
        .def("update", &ConstantStateExtendedKalmanFilter::update)
        .def_readwrite("state", &ConstantStateExtendedKalmanFilter::state)
        .def_readwrite("state_unc", &ConstantStateExtendedKalmanFilter::state_unc)
        .def_readwrite("jac", &ConstantStateExtendedKalmanFilter::jac)
        .def_readwrite("innovation", &ConstantStateExtendedKalmanFilter::innovation)
        .def_readwrite("innovation_unc", &ConstantStateExtendedKalmanFilter::innovation_unc)
        .def_readwrite("innovation_unc_inv", &ConstantStateExtendedKalmanFilter::innovation_unc_inv)
        .def_readwrite("gain", &ConstantStateExtendedKalmanFilter::gain)
        .def_readwrite("process_unc", &ConstantStateExtendedKalmanFilter::process_unc)
        .def_readwrite("measurement_unc", &ConstantStateExtendedKalmanFilter::measurement_unc);


#ifdef VERSION_INFO
    mod.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    mod.attr("__version__") = "dev";
#endif
}
// clang-format on