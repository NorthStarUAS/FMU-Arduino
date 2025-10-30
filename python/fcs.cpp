#include <pybind11/pybind11.h>
#include "src/fcs/ap.h"

namespace py = pybind11;

PYBIND11_MODULE(FCS, m) {
    m.doc() = "Simple C++ class wrapped with pybind11";

    py::class_<AutoPilot>(m, "AutoPilot")
        .def(py::init<>())
        .def("reset", &AutoPilot::reset)
        .def("update", &AutoPilot::update)
        .def("build", &AutoPilot::build);
}