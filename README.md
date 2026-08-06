# encos-sys-id

Linux library and tooling for **system identification of electric drives** over CAN.
Built to characterise ENCOS actuators for a humanoid robot: it drives a motor with
generated excitation signals, records the response over the CAN bus, and returns the
data to Python for parameter estimation. This work became the basis of my bachelor's
thesis.

## Highlights

- **Direct SocketCAN** — talks to the motor over a raw CAN socket
  (`PF_CAN` / `SOCK_RAW` / `CAN_RAW`), no vendor middleware.
- **Real-time signal generation** — the excitation thread runs under `SCHED_FIFO`,
  is pinned to a CPU core (`sched_setaffinity`), and paces messages with
  `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` to keep a steady send rate
  without jitter — important for clean identification data.
- **Python ↔ C++ ↔ hardware bridge** — the control signal is generated in Python,
  played onto the CAN bus by the C++ core, and the motor's response is returned to
  Python for analysis (pybind11).
- **Analysis notebooks** (`scripts/`): dataset generation for the MuJoCo (`mjx`)
  identification pipeline, and torque/inertia/friction measurements from a
  load-cell test stand.

## Stack

C++ · SocketCAN · Linux real-time scheduling (SCHED_FIFO) · pybind11 · Python · CMake

## Layout

- `src/` — C++ core: SocketCAN interface (`encos.cpp`), real-time generator
  (`generator.cpp`), Python bindings (`binding.cpp`)
- `scripts/` — Jupyter notebooks: dataset generation + test-stand measurements
