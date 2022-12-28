from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

__version__ = "0.0.1"

ext_modules = [
    Pybind11Extension("kalmanfilters",
        [
            "main.cpp",
            "src/linalg/linalg.cpp",
            "src/sensors/sensors.cpp",
            "src/extended_kalman_filter/ExtendedKalmanFilter.cpp",
            "src/extended_kalman_filter/ConstantPositionExtendedKalmanFilter.cpp",
            "src/extended_kalman_filter/ConstantVelocityExtendedKalmanFilterAccel.cpp",
            "src/extended_kalman_filter/ConstantVelocityExtendedKalmanFilterAccelGyro.cpp",
            "src/extended_kalman_filter/ConstantVelocityExtendedKalmanFilterAccelGyroMag.cpp"
        ],
        include_dirs = ["extern", "src"],
        define_macros = [('VERSION_INFO', __version__)],
        ),
]

setup(
    name="kalmanfilters",
    version=__version__,
    author="James Unicomb",
    author_email="james.unicomb@gmail.com",
    url="",
    description="A project to demonstrate kalman filters.",
    long_description="",
    ext_modules=ext_modules,
    extras_require={"test": "pytest"},
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.6",
)