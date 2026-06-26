#pragma once

#include <atomic>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "encos.hpp"

struct SignalGenerator {
  enum class Mode { Position, Velocity, Torque };

 private:
  Encos::Controller m_encos;
  uint8_t m_id;

  std::vector<float> m_pos;
  std::vector<float> m_spd;
  std::vector<float> m_cur;

  std::unique_ptr<std::runtime_error> m_exception;

  std::atomic<bool> m_do_brake;
  std::unique_ptr<std::thread> m_brake_thread = nullptr;

  Mode m_mode = Mode::Torque;
  float m_kmode = 0;

 public:
  /// @brief Ctor
  /// @param interface SocketCAN interface (typically "can0")
  /// @param id motor id
  SignalGenerator(const std::string& interface, uint8_t id);

  /// @brief Set signal mode: torque, position or velocity
  /// @param mode mode
  /// @param kmode corresponding constant (kp for position, kd for velocity).
  /// Value is ignored in torque mode
  void setMode(Mode mode, float kmode);

 private:
  void playSignalLoop(const std::vector<float>& signal, float dtms);
  static void playSignalRoutine(SignalGenerator* self,
                                const std::vector<float>& signal, float dtms);

  static void brakeRoutine(SignalGenerator* self, float kp, float kd);

 public:
  /// @brief Apply control signal and measure responce
  /// @param signal control signal (see `setMode`)
  /// @param dtms sample period in milliseconds
  /// @return responce: vector {position, velocity, current}
  std::vector<std::vector<float>> playSignal(const std::vector<float>& signal,
                                             float dtms);

  /// @brief Enable braking mode. Holds current position and zero velocity.
  /// @param kp 0 <= kp <= 500
  /// @param kd 0 <= kd <= 5
  void brake(float kp, float kd);

  /// @brief Disable braking mode
  void release();
};