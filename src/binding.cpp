#include "generator.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

PYBIND11_MODULE(PYTHON_MODULE_NAME, m) {
  m.doc() =
      "ENCOS drive identification tools.\n"
      "Author: Lekhterev V.V. @tndrd, WaveLab 2026";

  m.def("get_id", &Encos::Tools::getId);
  m.def("set_zero", &Encos::Tools::setZero);

  py::class_<SignalGenerator> gen(m, "SignalGenerator");

  gen.def(py::init<const std::string &, uint8_t>(),
          py::arg("interface") = "can0", py::arg("drive_id") = 1)
      .def("play_signal", &SignalGenerator::playSignal, py::arg("signal"),
           py::arg("dtms"))
      .def("play_pd_signal", &SignalGenerator::playPDSignal,
           py::arg("pos_signal"), py::arg("vel_signal"), py::arg("kp"),
           py::arg("kd"), py::arg("dtms"))
      .def("brake", &SignalGenerator::brake)
      .def("release", &SignalGenerator::release)
      .def("set_mode", &SignalGenerator::setMode, py::arg("mode"),
           py::arg("kmode") = 0)
      .def("set_factory_scales", &SignalGenerator::setFactoryScales);

  py::enum_<SignalGenerator::Mode>(gen, "mode")
      .value("position", SignalGenerator::Mode::Position)
      .value("velocity", SignalGenerator::Mode::Velocity)
      .value("torque", SignalGenerator::Mode::Torque);
}