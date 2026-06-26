#include "generator.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cassert>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::string_literals;

static void fatal(const std::string &msg, int err = 0) {
  throw std::runtime_error(msg + ((err != 0) ? ": "s + strerror(err) : ""));
}

static void schedSetup() {
  sched_param param;
  param.sched_priority = 80;

  if (sched_setscheduler(0, SCHED_FIFO, &param) < 0)
    fatal("sched_setscheduler", errno);

  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) fatal("mlockall", errno);

  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(0, &set);

  if (sched_setaffinity(0, sizeof(set), &set) < 0)
    fatal("sched_setaffinity", errno);
}

SignalGenerator::SignalGenerator(const std::string &interface, uint8_t id)
    : m_encos{interface}, m_id{id} {}

void SignalGenerator::setMode(Mode mode, float kmode) {
  m_mode = mode;
  m_kmode = kmode;
}

void SignalGenerator::playSignalLoop(const std::vector<float> &signal,
                                     float dtms) {
  timespec ts_next;
  clock_gettime(CLOCK_MONOTONIC, &ts_next);

  m_pos = std::vector<float>(signal.size(), 0);
  m_spd = std::vector<float>(signal.size(), 0);
  m_cur = std::vector<float>(signal.size(), 0);

  for (size_t i = 0; i < signal.size(); ++i) {
    float pos = (m_mode == Mode::Position) ? signal[i] : 0;
    float kp = (m_mode == Mode::Position) ? m_kmode : 0;

    float vel = (m_mode == Mode::Velocity) ? signal[i] : 0;
    float kd = (m_mode == Mode::Velocity) ? m_kmode : 0;

    float trq = (m_mode == Mode::Torque) ? signal[i] : 0;

    auto state = m_encos.hybridControl(m_id, kp, kd, pos, vel, trq);
    m_pos[i] = state.pos;
    m_spd[i] = state.spd;
    m_cur[i] = state.cur;

    ts_next.tv_nsec += long(dtms * 1000) * 1000;

    if (ts_next.tv_nsec >= 1000000000) {
      ts_next.tv_sec++;
      ts_next.tv_nsec -= 1000000000;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts_next, NULL);
  }

  m_encos.hybridControl(m_id, 0, 0, 0, 0, 0);
}

void SignalGenerator::playSignalRoutine(SignalGenerator *self,
                                        const std::vector<float> &signal,
                                        float dtms) {
  assert(self);
  self->m_exception = nullptr;
  try {
    schedSetup();
    self->playSignalLoop(signal, dtms);
  } catch (std::exception &e) {
    /// @todo properly catch the dynamic type of e
    self->m_exception = std::make_unique<std::runtime_error>(e.what());
  }
}

void SignalGenerator::brakeRoutine(SignalGenerator *self, float kp, float kd) {
  assert(self);
  self->m_exception = nullptr;

  auto state = self->m_encos.hybridControl(self->m_id, 0, 0, 0, 0, 0);

  try {
    while (self->m_do_brake) {
      self->m_encos.hybridControl(self->m_id, kp, kd, state.pos, 0, 0);
      usleep(20 * 1000);
    }
  } catch (std::exception &e) {
    /// @todo properly catch the dynamic type of e
    self->m_exception = std::make_unique<std::runtime_error>(e.what());
  }
}

std::vector<std::vector<float>> SignalGenerator::playSignal(
    const std::vector<float> &signal, float dtms) {
  std::thread thread(SignalGenerator::playSignalRoutine, this,
                     std::cref(signal), dtms);
  thread.join();

  if (m_exception.get()) throw *m_exception;

  return {m_pos, m_spd, m_cur};
}

void SignalGenerator::brake(float kp, float kd) {
  if (m_brake_thread.get()) release();

  m_do_brake = true;
  m_brake_thread = std::make_unique<std::thread>(SignalGenerator::brakeRoutine,
                                                 this, kp, kd);
}

void SignalGenerator::release() {
  m_do_brake = false;
  m_brake_thread->join();
  m_brake_thread = nullptr;
}