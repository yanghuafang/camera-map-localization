#ifndef CAM_LOC_TYPES_RANDOM_H_
#define CAM_LOC_TYPES_RANDOM_H_

/// Seeded sampling that gives the same answer on every standard library.

#include <cstdint>
#include <random>

namespace cam_loc {

/// Uniform and Gaussian draws from a `std::mt19937`, reproducibly.
///
/// The standard pins the *engine* -- `std::mt19937` emits a fixed sequence for
/// a given seed, everywhere -- and then leaves the *distributions* free to
/// consume that sequence however they like. libc++ and libstdc++ take
/// different numbers of draws and transform them differently, so
/// `std::normal_distribution` seeded identically yields different values on
/// macOS and on Linux.
///
/// That matters here more than it looks. A seeded map error is the difference
/// between a survey a test can assert against and one that is merely
/// plausible; if the survey differs by platform then so does every accuracy
/// figure taken against it, and a threshold tuned on one machine is noise on
/// the other. The engine is portable, so this reads it directly: a uniform by
/// fixed-point division, and a Gaussian by Box-Muller over two of those.
class Sampler {
 public:
  explicit Sampler(uint32_t seed) : engine_(seed) {}

  /// Uniform in [0, 1). Exactly `(x + ½) / 2³²` over the engine's raw output,
  /// which never returns 0 or 1 and so is safe to take a logarithm of.
  double Uniform();

  /// Standard normal. Box-Muller yields two values per pair of uniforms; the
  /// second is kept rather than discarded, so a run consumes the engine at a
  /// fixed rate.
  double Gaussian();

 private:
  std::mt19937 engine_;
  double spare_ = 0.0;
  bool has_spare_ = false;
};

}  // namespace cam_loc

#endif  // CAM_LOC_TYPES_RANDOM_H_
