#include "generator.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "tilkom_bindings.h"

namespace py = pybind11;

PYBIND11_MODULE(PYTHON_MODULE_NAME, m) {
  m.doc() =
      "ENCOS drive identification tools.\n"
      "Author: Lekhterev V.V. @tndrd, WaveLab 2026";

  m.def("get_id", &Encos::Tools::getId);
  m.def("set_zero", &Encos::Tools::setZero);

  py::class_<Encos::Scales::Values>(m, "Scales")
      .def_readwrite("kp", &Encos::Scales::Values::kp)
      .def_readwrite("kd", &Encos::Scales::Values::kd)
      .def_readwrite("pos", &Encos::Scales::Values::pos)
      .def_readwrite("vel", &Encos::Scales::Values::vel)
      .def_readwrite("trq", &Encos::Scales::Values::trq)
      .def_readwrite("cur", &Encos::Scales::Values::cur);

  py::class_<SignalGenerator> gen(m, "SignalGenerator");

  gen.def(py::init<const std::string &>(), py::arg("interface"))
      .def("play_signal", &SignalGenerator::playSignal, py::arg("inputs"),
           py::arg("dtms"))
      .def("gather", &SignalGenerator::gather);

  using Vec = std::vector<float>;
  py::class_<SignalGenerator::MotorInput>(gen, "MotorInput")
      .def(py::init<int, Encos::Scales::Values, Vec, Vec, Vec, Vec, Vec>(),
           py::arg("id"), py::arg("scales"), py::arg("pos"), py::arg("vel"),
           py::arg("trq"), py::arg("kp"), py::arg("kd"))
      .def_readwrite("id", &SignalGenerator::MotorInput::id)
      .def_readwrite("scales", &SignalGenerator::MotorInput::scales)
      .def_readwrite("pos", &SignalGenerator::MotorInput::pos)
      .def_readwrite("vel", &SignalGenerator::MotorInput::vel)
      .def_readwrite("trq", &SignalGenerator::MotorInput::trq)
      .def_readwrite("kp", &SignalGenerator::MotorInput::kp)
      .def_readwrite("kd", &SignalGenerator::MotorInput::kd);

  py::class_<SignalGenerator::MotorOutput>(gen, "MotorOutput")
      .def_readwrite("id", &SignalGenerator::MotorOutput::id)
      .def_readwrite("scales", &SignalGenerator::MotorOutput::scales)
      .def_readwrite("pos", &SignalGenerator::MotorOutput::pos)
      .def_readwrite("vel", &SignalGenerator::MotorOutput::vel)
      .def_readwrite("cur", &SignalGenerator::MotorOutput::cur)
      .def_readwrite("ts", &SignalGenerator::MotorOutput::ts)
      .def_readwrite("status", &SignalGenerator::MotorOutput::status);

  m.def("status_str", &Encos::Controller::Status::string);

  py::module_ scales_sub =
      m.def_submodule("scales", "Motor message encoding scales");
  scales_sub.attr("normalized") = Encos::Scales::normalized;
  scales_sub.attr("a8112_as_docs") = Encos::Scales::a8112AsDocs;
  scales_sub.attr("a8112_tuned") = Encos::Scales::a8112Tuned;

  tilkom::BindTilkom(m);
}