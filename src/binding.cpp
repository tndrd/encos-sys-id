#include "generator.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

PYBIND11_MODULE(PYTHON_MODULE_NAME, m) {
  m.doc() =
      "ENCOS drive identification tools.\n"
      "Author: Lekhterev V.V. @tndrd, WaveLab 2026";

  py::class_<SignalGenerator>(m, "SignalGenerator")
      .def(py::init<const std::string&, uint8_t>(),
           py::arg("interface") = "can0", py::arg("drive_id") = 1)

      .def("play_signal", &SignalGenerator::playSignal, py::arg("signal"),
           py::arg("dtms"));
}