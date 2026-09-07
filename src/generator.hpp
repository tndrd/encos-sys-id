#pragma once

#include <atomic>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>
#include <tuple>
#include <string>

#include "encos.hpp"

struct SignalGenerator {
  enum class Mode { Position, Velocity, Torque };

  using Responce = std::tuple<std::vector<float>,
                              std::vector<float>,
                              std::vector<float>,
                              std::vector<const char*>>;
 private:
  Encos::Controller m_encos;
  uint8_t m_id;

  std::vector<float> m_pos;
  std::vector<float> m_spd;
  std::vector<float> m_cur;
  std::vector<const char*> m_err;

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

  /// @brief Set factory scales for motor IO encoding/decoding
  /// @param use_factory_scale specifies whether a factory or a default scale should be used
  /// @note tuned scales are used by default
  void setFactoryScales(bool use_factory_scale);

 private:
  void playSignalLoop(const std::vector<float>& signal, float dtms);
  void playPDSignalLoop(const std::vector<float>& pos_signal,
                        const std::vector<float>& vel_signal, float kp,
                        float kd, float dtms);
  static void playSignalRoutine(SignalGenerator* self,
                                const std::vector<float>& signal, float dtms);
  static void playPDSignalRoutine(SignalGenerator* self,
                                  const std::vector<float>& pos_signal,
                                  const std::vector<float>& vel_signal,
                                  float kp, float kd, float dtms);

  static void brakeRoutine(SignalGenerator* self, float kp, float kd);

 public:
  /// @brief Apply control signal and measure responce
  /// @param signal control signal (see `setMode`)
  /// @param dtms sample period in milliseconds
  /// @return responce: tuple {position, velocity, current, string}
  Responce playSignal(const std::vector<float>& signal,
                                             float dtms);

  /// @brief Apply position/velocity targets with fixed PD gains and measure response
  /// @param pos_signal target position signal, rad
  /// @param vel_signal target velocity signal, rad/s
  /// @param kp position gain, 0 <= kp <= 500
  /// @param kd velocity gain, 0 <= kd <= 10
  /// @param dtms sample period in milliseconds
  /// @return responce: tuple {position, velocity, current, error}
  Responce playPDSignal(
      const std::vector<float>& pos_signal,
      const std::vector<float>& vel_signal, float kp, float kd, float dtms);

  /// @brief Enable braking mode. Holds current position and zero velocity.
  /// @param kp 0 <= kp <= 500
  /// @param kd 0 <= kd <= 5
  void brake(float kp, float kd);

  /// @brief Disable braking mode
  void release();
};