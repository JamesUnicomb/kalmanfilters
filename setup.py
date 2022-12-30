from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

__version__ = "0.0.1"

ext_modules = [
    Pybind11Extension(
        "kalmanfilters",
        [
            "src/linalg/linalg.cpp",
            "src/sensors/sensors.cpp",
            "src/models/ConstantPositionAccel.cpp",
            "src/models/ConstantVelocityAccel.cpp",
            "src/models/ConstantVelocityAccelGyro.cpp",
            "src/models/ConstantVelocityAccelGyroMag.cpp",
            "main.cpp",
        ],
        include_dirs=["extern", "src"],
        define_macros=[("VERSION_INFO", __version__)],
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
