#ifndef CAM_LOC_TYPES_STATUS_CODES_H_
#define CAM_LOC_TYPES_STATUS_CODES_H_

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

}  // namespace cam_loc

#endif  // CAM_LOC_TYPES_STATUS_CODES_H_
