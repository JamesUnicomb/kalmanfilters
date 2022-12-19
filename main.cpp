#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

#include "nr3.hpp"
#include "extended_kalman_filter/ConstantStateExtendedKalmanFilter.hpp"

// clang-format off
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

using namespace std;
namespace py = pybind11;

PYBIND11_MODULE(kalmanfilters, mod) {
    mod.doc() = "";

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