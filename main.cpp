#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

#include "nr3.hpp"
#include "linalg/linalg.hpp"
#include "quaternion/quaternion.hpp"
#include "sensors/sensors.hpp"
#include "kalmanfilter/ExtendedKalmanFilter.hpp"
// #include "kalmanfilter/UnscentedKalmanFilter.hpp"
#include "models/ConstantPositionAccel.hpp"
#include "models/ConstantPositionAccelMagQuat.hpp"
// #include "models/ConstantVelocityAccelGyro.hpp"
// #include "models/ConstantVelocityAccelGyroMag.hpp"
#include "models/ConstantVelocityAccelGyroMagQuat.hpp"

// clang-format off
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

using namespace std;
using namespace linalg;
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

    py::module quaternion = mod.def_submodule("quaternion", "misc. quaternion utilities.");
    quaternion.def("q_to_euler", &quaternion::q_to_euler);
    quaternion.def("q_to_mat4", &quaternion::q_to_mat4);

    py::module linalg = mod.def_submodule("linalg", "misc. linear algebra utilities.");
    py::class_<Vector>(linalg, "Vector")
        .def(py::init<vector<double>&>())
        .def("tovec", &Vector::tovec);
    py::class_<Matrix>(linalg, "Matrix")
        .def(py::init<vector<vector<double>>&>())
        .def("tovec", &Matrix::tovec);

    typedef ExtendedKalmanFilter<ConstantPositionAccelMotionModel, ConstantPositionAccelMeasurementModel> cpekf;
    py::class_<cpekf>(mod, "cpekf")
        .def(py::init<double, Vector, Matrix>())
        .def("set_state", &cpekf::set_state)
        .def("set_state_unc", &cpekf::set_state_unc)
        .def("get_state", &cpekf::get_state)
        .def("get_state_unc", &cpekf::get_state_unc)
        .def("get_innovation", &cpekf::get_innovation)
        .def("get_innovation_unc", &cpekf::get_innovation_unc)
        .def("predict", &cpekf::predict)
        .def("update", &cpekf::update<sensors::accel&>);

    typedef ExtendedKalmanFilter<ConstantPositionAccelMagQuatMotionModel, ConstantPositionAccelMagQuatMeasurementModel> cpqekf;
    py::class_<cpqekf>(mod, "cpqekf")
        .def(py::init<double, Vector, Matrix>())
        .def("set_state", &cpqekf::set_state)
        .def("set_state_unc", &cpqekf::set_state_unc)
        .def("get_state", &cpqekf::get_state)
        .def("get_state_unc", &cpqekf::get_state_unc)
        .def("get_innovation", &cpqekf::get_innovation)
        .def("get_innovation_unc", &cpqekf::get_innovation_unc)
        .def("predict", &cpqekf::predict)
        .def("update", &cpqekf::update<sensors::accel&>)
        .def("update", &cpqekf::update<sensors::mag&>);

    // typedef ExtendedKalmanFilter<ConstantVelocityAccelGyroMagMotionModel, ConstantVelocityAccelGyroMagMeasurementModel> cvekf;
    // py::class_<cvekf>(mod, "cvekf")
    //     .def(py::init<double, vector<double>, vector<vector<double>>>())
    //     .def("predict", &cvekf::predict)
    //     .def("update", &cvekf::update<sensors::accel&>)
    //     .def("update", &cvekf::update<sensors::gyro&>)
    //     .def("update", &cvekf::update<sensors::mag&>)
    //     .def_readwrite("state", &cvekf::state)
    //     .def_readwrite("state_unc", &cvekf::state_unc)
    //     .def_readwrite("innovation", &cvekf::innovation)
    //     .def_readwrite("innovation_unc", &cvekf::innovation_unc)
    //     .def_readwrite("dhdx", &cvekf::dhdx);

    // typedef UnscentedKalmanFilter<ConstantVelocityAccelGyroMagMotionModel, ConstantVelocityAccelGyroMagMeasurementModel> cvukf;
    // py::class_<cvukf>(mod, "cvukf")
    //     .def(py::init<double, vector<double>, vector<vector<double>>>())
    //     .def("predict", &cvukf::predict)
    //     .def("update", &cvukf::update<sensors::accel&>)
    //     .def("update", &cvukf::update<sensors::gyro&>)
    //     .def("update", &cvukf::update<sensors::mag&>)
    //     .def_readwrite("state", &cvukf::state)
    //     .def_readwrite("state_unc", &cvukf::state_unc)
    //     .def_readwrite("state_unc_sqrtm", &cvukf::state_unc_sqrtm)
    //     .def_readwrite("innovation", &cvukf::innovation)
    //     .def_readwrite("innovation_unc", &cvukf::innovation_unc)
    //     .def_readwrite("state_sigma_points", &cvukf::state_sigma_points)
    //     .def_readwrite("measurement_sigma_points", &cvukf::measurement_sigma_points);

    typedef ExtendedKalmanFilter<ConstantVelocityAccelGyroMagQuatMotionModel, ConstantVelocityAccelGyroMagQuatMeasurementModel> cvqekf;
    py::class_<cvqekf>(mod, "cvqekf")
        .def(py::init<double, Vector, Matrix>())
        .def("set_state", &cvqekf::set_state)
        .def("set_state_unc", &cvqekf::set_state_unc)
        .def("get_state", &cvqekf::get_state)
        .def("get_state_unc", &cvqekf::get_state_unc)
        .def("get_innovation", &cvqekf::get_innovation)
        .def("get_innovation_unc", &cvqekf::get_innovation_unc)
        .def("predict", &cvqekf::predict)
        .def("update", &cvqekf::update<sensors::accel&>)
        .def("update", &cvqekf::update<sensors::gyro&>)
        .def("update", &cvqekf::update<sensors::mag&>);


#ifdef VERSION_INFO
    mod.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    mod.attr("__version__") = "dev";
#endif
}
// clang-format on