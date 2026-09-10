#pragma once

namespace Encos {
namespace Scales {
struct Values {
  float kp, kd, pos, vel, trq, cur;
};

static constexpr Values normalized = {
    1.f,  // kp
    1.f,  // kd
    1.f,  // pos
    1.f,  // vel
    1.f,  // trq
    1.f,  // cur
};

static constexpr Values a8112AsDocs = {
    500.f,  // kp
    5.0f,   // kd
    12.5f,  // pos
    18.0f,  // vel
    90.0f,  // trq
    30.0f,  // cur
};

static constexpr Values a8112Tuned = {
    500.f,  // kp
    10.0f,  // kd
    10.0f,  // pos
    20.0f,  // vel
    90.0f,  // trq
    30.0f,  // cur
};

}  // namespace Scales
}  // namespace Encos