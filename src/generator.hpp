#pragma once

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "encos.hpp"

struct SignalGenerator {
 public:
  struct MotorInput {
    uint8_t id;
    Encos::Scales::Values scales;
    std::vector<float> pos;
    std::vector<float> vel;
    std::vector<float> trq;
    std::vector<float> kp;
    std::vector<float> kd;
  };

  struct MotorOutput {
    uint8_t id;
    Encos::Scales::Values scales;
    std::vector<float> pos;
    std::vector<float> vel;
    std::vector<float> cur;
    std::vector<uint64_t> ts;
    // Integer type to propagate enum to python
    std::vector<uint8_t> status;
  };

 private:
  Encos::Controller m_encos;
  std::unique_ptr<std::runtime_error> m_exception;
  std::unique_ptr<std::thread> m_thread;
  std::vector<MotorOutput> m_outputs;
  std::vector<MotorInput> m_inputs;

 public:
  /// @brief Ctor
  /// @param interface SocketCAN interface (typically "can0")
  /// @param id motor id
  SignalGenerator(const std::string& interface);

 private:
  void playSignalLoop(const std::vector<MotorInput>& inputs, float dtms);
  static void playSignalRoutine(SignalGenerator* self,
                                const std::vector<MotorInput>& inputs,
                                float dtms);

 public:
  /// @brief Apply control signal and measure responce. Starts separate thread
  /// and returns immediately
  /// @param signal control signal (see `setMode`)
  /// @param dtms sample period in milliseconds
  void playSignal(const std::vector<MotorInput>& inputs, float dtms);

  std::vector<MotorOutput> gather();
};