#include "encos.hpp"

#include <linux/can.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cassert>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace Encos {
namespace Helpers {

void write(int fd, const uint8_t* data, size_t sz) {
  assert(data);
  size_t i = 0;

  while (i < sz) {
    int ret = ::write(fd, data + i, sz - i);
    if (ret < 0)
      throw std::runtime_error(std::string("write(): ") + strerror(errno));
    i += ret;
  }
}

void read(int fd, uint8_t* data, size_t sz) {
  assert(data);
  size_t i = 0;

  while (i < sz) {
    int ret = ::read(fd, data + i, sz - i);
    if (ret < 0)
      throw std::runtime_error(std::string("read(): ") + strerror(errno));
    i += ret;
  }
}

uint16_t float2uint(float x, float x_min, float x_max, int bits) {
  if (x > x_max) x = x_max;
  if (x < x_min) x = x_min;
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint2float(uint16_t x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int + .5f) * span / ((float)((1 << bits) - 1)) + offset;
}
}  // namespace Helpers

void Descriptor::Deleter::operator()(int* pdescr) {
  if (!pdescr) return;
  close(*pdescr);
  delete pdescr;
}

Descriptor::Descriptor(int fd) : m_ptr{new int(fd)} {}
const int& Descriptor::operator*() const { return *m_ptr; }

Descriptor CANInterface::makeSocket(const std::string& interface) {
  using namespace std::string_literals;

  // Open socket
  Descriptor fd{socket(PF_CAN, SOCK_RAW, CAN_RAW)};
  if (*fd < 0) throw std::runtime_error("socket(): "s + strerror(errno));

  // Set timeout
  timeval tv;
  tv.tv_sec = timeoutMs / 1000;
  tv.tv_usec = (timeoutMs % 1000) * 1000;
  if (setsockopt(*fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    throw std::runtime_error("setsockopt(): "s + strerror(errno));

  // Get interface index
  ifreq ifr;
  strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
  if (ioctl(*fd, SIOCGIFINDEX, &ifr) < 0)
    throw std::runtime_error("ioctl(): "s + strerror(errno));

  // Bind socket to interface
  sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(*fd, (sockaddr*)(&addr), sizeof(addr)) < 0)
    throw std::runtime_error("bind(): "s + strerror(errno));

  return fd;
}

CANInterface::CANInterface(const std::string& interface)
    : m_socket{makeSocket(interface)} {}

void CANInterface::send(uint32_t id, uint8_t len, const uint8_t* data) {
  assert(data);
  assert(len <= 8);

  can_frame frame;
  memset(&frame, 0, sizeof(frame));

  frame.can_id = id;
  frame.len = len;
  memcpy(frame.data, data, len);

  Helpers::write(*m_socket, reinterpret_cast<uint8_t*>(&frame), sizeof(frame));
}

void CANInterface::recv(uint32_t* id, uint8_t* len, uint8_t* data) {
  assert(id);
  assert(len);
  assert(data);

  can_frame frame;
  Helpers::read(*m_socket, reinterpret_cast<uint8_t*>(&frame), sizeof(frame));

  *id = frame.can_id;
  *len = frame.len;

  memcpy(data, frame.data, frame.len);
}

uint16_t Tools::getId(const std::string& interface) {
  CANInterface can{interface};

  uint8_t data[8] = {0xff, 0xff, 0x00, 0x82};
  can.send(0x7ff, 4, data);

  uint32_t id;
  uint8_t len;
  can.recv(&id, &len, data);

  assert(id == 0x7ff);
  if (len != 5) throw std::runtime_error("getId: error responce");

  std::swap(data[3], data[4]);
  uint16_t respId = *reinterpret_cast<uint16_t*>(&data[3]);
  std::cerr << "Found id #" << respId << std::endl;
  return respId;
}

void Tools::setZero(const std::string& interface) {
  uint16_t id = Tools::getId(interface);
  uint8_t* idByte = reinterpret_cast<uint8_t*>(&id);
  std::swap(idByte[0], idByte[1]);

  CANInterface can{interface};
  uint8_t cmd[4] = {idByte[0], idByte[1], 0x00, 0x03};

  can.send(0x7ff, 4, cmd);

  uint8_t len;
  uint32_t id2;
  can.recv(&id2, &len, cmd);

  assert(id2 == 0x7ff);
  assert(len == 4);
  assert(cmd[0] == idByte[0] && cmd[1] == idByte[1]);
  assert(cmd[3] == 0x03 && "Drive refused to set zero");
}

const char* Controller::Status::string() const {
  switch (code) {
    // clang-format off
    #define CASE(x) case x: return #x
    CASE(NoError);
    CASE(OverTemperature);
    CASE(OverCurrent);
    CASE(UnderVoltage);
    CASE(EncoderError);
    CASE(BrakeOverVoltage);
    CASE(DriverError);
    default: return "UnknownError";
    #undef case
      // clang-format on
  }
}

Controller::Controller(const std::string& interface) : m_can{interface} {}

auto Controller::hybridControl(uint32_t id, Scales::Values scales, Command cmd)
    -> State {
  using namespace Helpers;

  uint16_t p_int = float2uint(cmd.pos, -scales.pos, scales.pos, 16);
  uint16_t v_int = float2uint(cmd.vel, -scales.vel, scales.vel, 12);
  uint16_t kp_int = float2uint(cmd.kp, 0, scales.kp, 12);
  uint16_t kd_int = float2uint(cmd.kd, 0, scales.kd, 9);
  uint16_t t_int = float2uint(cmd.trq, -scales.trq, scales.trq, 12);

  uint8_t data[8];
  data[0] = (kp_int >> 7) & 0x1F;
  data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8);
  data[2] = kd_int & 0xFF;
  data[3] = p_int >> 8;
  data[4] = p_int & 0xFF;
  data[5] = v_int >> 4;
  data[6] = ((v_int & 0xF) << 4) | (t_int >> 8);
  data[7] = t_int & 0xFF;

  m_can.send(id, 8, data);

  uint32_t respId;
  uint8_t len;
  m_can.recv(&respId, &len, data);

  assert(id == respId);
  assert(len == 8);
  assert((data[0] & 0b11100000) == 0b00100000);

  State state;

  assert(data[0] == 0b00100000);

  state.status.code =
      static_cast<Encos::Controller::Status::Type>(data[0] & 0x1F);

  p_int = (data[1] << 8) | data[2];
  state.pos = uint2float(p_int, -scales.pos, scales.pos, 16);

  v_int = (data[3] << 4) | (data[4] >> 4);
  state.vel = uint2float(v_int, -scales.vel, scales.vel, 12);

  t_int = ((data[4] & 0xF) << 8) | data[5];
  state.cur = uint2float(t_int, -scales.cur, scales.cur, 12);

  return state;
}

}  // namespace Encos