#pragma once

/// Lightweight result codes returned by I/O, map, perception, and CUDA paths.

namespace cam_loc {

enum class Status {
  kOk = 0,
  kInvalidArgument,
  kIoError,
  kNotFound,
  kNotImplemented,
  kCudaError,
};

inline const char* toString(Status s);

}  // namespace cam_loc
