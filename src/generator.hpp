#pragma once

#include <stdexcept>
#include <vector>
#include <memory>
#include <utility>

#include "encos.hpp"

struct SignalGenerator {
 private:
  Encos::Controller m_encos;
  uint8_t m_id;

  std::vector<float> m_pos;
  std::vector<float> m_spd;
  std::unique_ptr<std::runtime_error> m_exception;

 public:
  SignalGenerator(const std::string& interface, uint8_t id);

 private:
  void playSignalLoop(const std::vector<float>& signal, float dtms);
  static void playSignalRoutine(SignalGenerator *self, const std::vector<float>& signal, float dtms);

 public:
  std::pair<std::vector<float>, std::vector<float>> playSignal(const std::vector<float>& signal, float dtms);
};
