cmake_minimum_required(VERSION 3.15)
project(nstFMU LANGUAGES CXX)

find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

# FCS module
pybind11_add_module(FCS
    fcs.cpp
    src/fcs/ap.cpp
    src/fcs/pid.cpp
    src/fcs/tecs.cpp
    src/props2.cpp
    src/util/strutils.cpp
)

# Install the built module into the Python package
install(TARGETS FCS DESTINATION nstFMU)