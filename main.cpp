#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

#include "nr3.hpp"
#include "linalg/linalg.hpp"
#include "quaternion/quaternion.hpp"
#include "sensors/sensors.hpp"
#include "igrf/igrf.hpp"
#include "kalmanfilter/ExtendedKalmanFilter.hpp"
#include "kalmanfilter/UnscentedKalmanFilter.hpp"
#include "models/ConstantPositionAccel.hpp"
#include "models/ConstantPositionAccelMagQuat.hpp"
#include "models/ConstantVelocityAccelGyroMag.hpp"
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
        .def("vec", &sensors::accel::vec)
        .def("unc", &sensors::accel::unc);
    py::class_<sensors::gyro>(sensors, "gyro")
        .def(py::init<double, double, double>())
        .def(py::init<double, double, double, double, double, double>())
        .def("vec", &sensors::gyro::vec)
        .def("unc", &sensors::gyro::unc);
    py::class_<sensors::mag>(sensors, "mag")
        .def(py::init<double, double, double>())
        .def(py::init<double, double, double, double, double, double>())
        .def("vec", &sensors::mag::vec)
        .def("unc", &sensors::mag::unc);

    py::module quaternion = mod.def_submodule("quaternion", "misc. quaternion utilities.");
    quaternion.def("q_to_euler", py::overload_cast<Vector&>(&quaternion::q_to_euler));
    quaternion.def("q_to_mat4", py::overload_cast<Vector&>(&quaternion::q_to_mat4));

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

    typedef UnscentedKalmanFilter<ConstantPositionAccelMagQuatMotionModel, ConstantPositionAccelMagQuatMeasurementModel> cpqukf;
    py::class_<cpqukf>(mod, "cpqukf")
        .def(py::init<double, Vector, Matrix>())
        .def("set_state", &cpqukf::set_state)
        .def("set_state_unc", &cpqukf::set_state_unc)
        .def("get_state", &cpqukf::get_state)
        .def("get_state_unc", &cpqukf::get_state_unc)
        .def("get_innovation", &cpqukf::get_innovation)
        .def("get_innovation_unc", &cpqukf::get_innovation_unc)
        .def("predict", &cpqukf::predict)
        .def("update", &cpqukf::update<sensors::accel&>)
        .def("update", &cpqukf::update<sensors::mag&>);

    typedef ExtendedKalmanFilter<ConstantVelocityAccelGyroMagMotionModel, ConstantVelocityAccelGyroMagMeasurementModel> cvekf;
    py::class_<cvekf>(mod, "cvekf")
        .def(py::init<double, Vector, Matrix>())
        .def("set_state", &cvekf::set_state)
        .def("set_state_unc", &cvekf::set_state_unc)
        .def("get_state", &cvekf::get_state)
        .def("get_state_unc", &cvekf::get_state_unc)
        .def("get_innovation", &cvekf::get_innovation)
        .def("get_innovation_unc", &cvekf::get_innovation_unc)
        .def("predict", &cvekf::predict)
        .def("update", &cvekf::update<sensors::accel&>)
        .def("update", &cvekf::update<sensors::gyro&>)
        .def("update", &cvekf::update<sensors::mag&>);

    typedef UnscentedKalmanFilter<ConstantVelocityAccelGyroMagQuatMotionModel, ConstantVelocityAccelGyroMagQuatMeasurementModel> cvqukf;
    py::class_<cvqukf>(mod, "cvqukf")
        .def(py::init<double, Vector, Matrix>())
        .def("set_state", &cvqukf::set_state)
        .def("set_state_unc", &cvqukf::set_state_unc)
        .def("get_state", &cvqukf::get_state)
        .def("get_state_unc", &cvqukf::get_state_unc)
        .def("get_innovation", &cvqukf::get_innovation)
        .def("get_innovation_unc", &cvqukf::get_innovation_unc)
        .def("predict", &cvqukf::predict)
        .def("update", &cvqukf::update<sensors::accel&>)
        .def("update", &cvqukf::update<sensors::gyro&>)
        .def("update", &cvqukf::update<sensors::mag&>);

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