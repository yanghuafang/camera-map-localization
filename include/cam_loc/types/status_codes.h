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

/// Human-readable label for a Status value (logging / CLI output).
inline const char* ToString(Status s) {
  switch (s) {
    case Status::kOk:
      return "Ok";
    case Status::kInvalidArgument:
      return "InvalidArgument";
    case Status::kIoError:
      return "IoError";
    case Status::kNotFound:
      return "NotFound";
    case Status::kNotImplemented:
      return "NotImplemented";
    case Status::kCudaError:
      return "CudaError";
  }
  return "Unknown";
}

}  // namespace cam_loc

#endif  // CAM_LOC_TYPES_STATUS_CODES_H_
