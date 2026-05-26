#pragma once

#include <stdexcept>
#include <vector>
#include <memory>
#include <utility>
#include <atomic>
#include <thread>

#include "encos.hpp"

struct SignalGenerator {
 private:
  Encos::Controller m_encos;
  uint8_t m_id;

  std::vector<float> m_pos;
  std::vector<float> m_spd;
  std::vector<float> m_cur;

  std::unique_ptr<std::runtime_error> m_exception;

  std::atomic<bool> m_do_brake;
  std::unique_ptr<std::thread> m_brake_thread = nullptr;

  int m_mode = 2;

 public:
  SignalGenerator(const std::string& interface, uint8_t id);

  void setMode(int mode);

 private:
  void playSignalLoop(const std::vector<float>& signal, float dtms);
  static void playSignalRoutine(SignalGenerator *self, const std::vector<float>& signal, float dtms);

  static void brakeRoutine(SignalGenerator* self, float kp, float kd);
 
 public:
  std::vector<std::vector<float>> playSignal(const std::vector<float>& signal, float dtms);
  void brake(float kp, float kd);
  void release();
};