// Portable seeded sampling: uniform by fixed-point division, Gaussian by
// Box-Muller.

#include "cam_loc/types/random.h"

#include <cmath>

namespace cam_loc {

namespace {
// 2^32, the span of std::mt19937's output.
constexpr double kEngineSpan = 4294967296.0;
}  // namespace

double Sampler::Uniform() {
  return (static_cast<double>(engine_()) + 0.5) / kEngineSpan;
}

double Sampler::Gaussian() {
  if (has_spare_) {
    has_spare_ = false;
    return spare_;
  }
  const double radius = std::sqrt(-2.0 * std::log(Uniform()));
  const double angle = 2.0 * M_PI * Uniform();
  spare_ = radius * std::sin(angle);
  has_spare_ = true;
  return radius * std::cos(angle);
}

}  // namespace cam_loc
