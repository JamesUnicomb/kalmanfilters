#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

// clang-format off
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

using namespace std;
namespace py = pybind11;

PYBIND11_MODULE(kalmanfilters, mod) {
    mod.doc() = "";

#ifdef VERSION_INFO
    mod.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    mod.attr("__version__") = "dev";
#endif
}
// clang-format on