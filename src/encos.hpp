#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace Encos {

// RAII wrapper for a file descriptor
class Descriptor final {
 private:
  struct Deleter final {
    void operator()(int* pdescr);
  };

  using Ptr = std::unique_ptr<int, Deleter>;

 private:
  Ptr m_ptr;

 public:
  explicit Descriptor(int fd = -1);
  const int& operator*() const;
};

// SocketCAN interface
struct CANInterface {
 private:
  static constexpr uint16_t timeoutMs = 1000;

 private:
  Descriptor m_socket;

 private:
  static Descriptor makeSocket(const std::string& interface);

 public:
  CANInterface(const std::string& interface);

  void send(uint32_t id, uint8_t len, const uint8_t* data);
  void recv(uint32_t* id, uint8_t* len, uint8_t* data);
};

namespace Tools {
/// @brief Request ID of a motor
/// @param interface interface name (typically ```"can0"```)
/// @note Would not work if several motors are present on bus
uint16_t getId(const std::string& interface);

/// @brief Set zero of a single motor
/// @param interface interface name (typically ```"can0"```)
/// @note Would not work if several motors are present on bus
void setZero(const std::string& interface);
}  // namespace Tools

struct Scales {
  float kp, kd, pos, vel, trq, cur;

  /// @brief Get scales by motor documentation
  /// @return Standard scales
  static Scales factory();

  /// @brief Get tuned scales (applicable for most older motors)
  /// @return Tuned scales
  static Scales tuned();
};

// Encos controller
struct Controller {
 public:
  struct Error {
    enum Type: uint8_t {
      // clang-format off
      NoError          = 0,
      OverTemperature  = 1,
      OverCurrent      = 2,
      UnderVoltage     = 3,
      EncoderError     = 4,
      BrakeOverVoltage = 6,
      DriverError      = 7
      // clang-format on
    } code;

    const char* string() const;
  };

  struct State {
    float pos;  // rad
    float spd;  // rad/s
    float cur;  // A
    Error err;
  };

 private:
  CANInterface m_can;

 public:
  Scales m_scales = Scales::tuned();

 public:
  /// @brief Create a drive interface
  /// @param interface interface name (typically ```"can0"```)
  Controller(const std::string& interface);

  /// @brief Hybrid control of a drive.
  /// @note  ```Torque = kp*(pos - actual_pos) + kd*(spd - actual_spd) + trq```
  /// @param id CAN id of a motor
  /// @param kp Coefficient, ```0 <= kp <= 500```
  /// @param kd Coefficient, ```0 <= kd <= 5```
  /// @param pos Radians, ```-12.5 <= pos <= 12.5```
  /// @param vel Radians/s, ```-18 <= vel <= 18```
  /// @param trq Nm, ```-30 <= trq <= 30```
  /// @return Current state (rad, rad/s)
  State hybridControl(uint32_t id, float kp, float kd, float pos, float vel,
                      float trq);
};

}  // namespace Encos