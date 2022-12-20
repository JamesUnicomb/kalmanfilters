#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

#include "nr3.hpp"
#include "sensors/sensors.hpp"
#include "extended_kalman_filter/ConstantPositionExtendedKalmanFilter.hpp"
#include "extended_kalman_filter/ConstantVelocityExtendedKalmanFilter.hpp"

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

    py::class_<ConstantPositionExtendedKalmanFilter>(mod, "ConstantPositionExtendedKalmanFilter")
        .def(py::init<double, double>())
        .def("predict", &ConstantPositionExtendedKalmanFilter::predict)
        .def("update", &ConstantPositionExtendedKalmanFilter::update)
        .def_readwrite("state", &ConstantPositionExtendedKalmanFilter::state)
        .def_readwrite("state_unc", &ConstantPositionExtendedKalmanFilter::state_unc)
        .def_readwrite("jac", &ConstantPositionExtendedKalmanFilter::jac)
        .def_readwrite("innovation", &ConstantPositionExtendedKalmanFilter::innovation)
        .def_readwrite("innovation_unc", &ConstantPositionExtendedKalmanFilter::innovation_unc)
        .def_readwrite("innovation_unc_inv", &ConstantPositionExtendedKalmanFilter::innovation_unc_inv)
        .def_readwrite("gain", &ConstantPositionExtendedKalmanFilter::gain)
        .def_readwrite("process_unc", &ConstantPositionExtendedKalmanFilter::process_unc)
        .def_readwrite("measurement_unc", &ConstantPositionExtendedKalmanFilter::measurement_unc);

    py::class_<ConstantVelocityExtendedKalmanFilter>(mod, "ConstantVelocityExtendedKalmanFilter")
        .def(py::init<double>())
        .def("predict", &ConstantVelocityExtendedKalmanFilter::predict)
        .def("update", py::overload_cast<const sensors::accel&, double>(&ConstantVelocityExtendedKalmanFilter::update))
        .def("update", py::overload_cast<const sensors::gyro&, double>(&ConstantVelocityExtendedKalmanFilter::update))
        .def_readwrite("state", &ConstantVelocityExtendedKalmanFilter::state)
        .def_readwrite("state_unc", &ConstantVelocityExtendedKalmanFilter::state_unc)
        .def_readwrite("jac", &ConstantVelocityExtendedKalmanFilter::jac)
        .def_readwrite("innovation", &ConstantVelocityExtendedKalmanFilter::innovation)
        .def_readwrite("innovation_unc", &ConstantVelocityExtendedKalmanFilter::innovation_unc)
        .def_readwrite("innovation_unc_inv", &ConstantVelocityExtendedKalmanFilter::innovation_unc_inv)
        .def_readwrite("gain", &ConstantVelocityExtendedKalmanFilter::gain)
        .def_readwrite("process_unc", &ConstantVelocityExtendedKalmanFilter::process_unc);

#ifdef VERSION_INFO
    mod.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    mod.attr("__version__") = "dev";
#endif
}
// clang-format on