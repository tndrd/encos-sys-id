#include "generator.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cassert>
#include <cstring>
#include <iostream>
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

SignalGenerator::SignalGenerator(const std::string &interface)
    : m_encos{interface} {}

void SignalGenerator::playSignalLoop(const std::vector<MotorInput> &inputs,
                                     float dtms) {
  assert(inputs.size() > 0);
  size_t size = inputs[0].pos.size();

  m_outputs = {};
  for (const auto &input : inputs) {
    MotorOutput output;

    output.id = input.id;
    output.scales = input.scales;
    output.pos = std::vector<float>(size, 0);
    output.vel = std::vector<float>(size, 0);
    output.cur = std::vector<float>(size, 0);
    output.ts = std::vector<uint64_t>(size, 0);

    output.status =
        std::vector<uint8_t>(size, Encos::Controller::Status::NoError);

    m_outputs.emplace_back(output);
  }

  timespec ts_next;
  clock_gettime(CLOCK_MONOTONIC, &ts_next);

  for (size_t isample = 0; isample < size; ++isample) {
    for (int imotor = 0; imotor < inputs.size(); ++imotor) {
      const auto &input = inputs[imotor];
      auto &output = m_outputs[imotor];

      Encos::Controller::Command cmd{input.kp[isample], input.kd[isample],
                                     input.pos[isample], input.vel[isample],
                                     input.trq[isample]};

      Encos::Controller::State state =
          m_encos.hybridControl(input.id, input.scales, cmd);

      output.pos[isample] = state.pos;
      output.vel[isample] = state.vel;
      output.cur[isample] = state.cur;

      timespec ts_now;
      clock_gettime(CLOCK_MONOTONIC, &ts_now);

      output.ts[isample] = ts_now.tv_sec * 1000000000 + ts_now.tv_nsec;
      output.status[isample] = state.status.code;
    }

    ts_next.tv_nsec += long(dtms * 1000) * 1000;

    if (ts_next.tv_nsec >= 1000000000) {
      ts_next.tv_sec++;
      ts_next.tv_nsec -= 1000000000;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts_next, NULL);
  }
}

void SignalGenerator::playSignalRoutine(SignalGenerator *self,
                                        const std::vector<MotorInput> &inputs,
                                        float dtms) {
  assert(self);
  self->m_exception = nullptr;
  try {
    schedSetup();
    self->playSignalLoop(inputs, dtms);
  } catch (std::exception &e) {
    /// @todo properly catch the dynamic type of e
    self->m_exception = std::make_unique<std::runtime_error>(e.what());
  }
}

static void ensureSize(const std::vector<float> &a, size_t sz) {
  if (a.size() != sz) throw std::runtime_error("Array size mismatch");
}

void SignalGenerator::playSignal(const std::vector<MotorInput> &inputs,
                                 float dtms) {
  if (m_thread.get()) throw std::runtime_error("SignalGenerator is busy");
  if (inputs.empty()) throw std::runtime_error("No inputs provided");

  // Ensure all inputs have same sizes
  size_t size = inputs[0].pos.size();  // arbitrary input

  for (const auto &input : inputs) {
    ensureSize(input.kd, size);
    ensureSize(input.kp, size);
    ensureSize(input.pos, size);
    ensureSize(input.trq, size);
    ensureSize(input.vel, size);
  }

  m_inputs = inputs;

  m_exception = nullptr;
  m_thread = std::make_unique<std::thread>(SignalGenerator::playSignalRoutine,
                                           this, std::cref(m_inputs), dtms);
}

std::vector<SignalGenerator::MotorOutput> SignalGenerator::gather() {
  if (!m_thread.get())
    throw std::runtime_error("SignalGenerator is not running");

  m_thread->join();
  m_thread = nullptr;

  // todo rollback

  if (m_exception.get()) throw *m_exception;

  return m_outputs;
}