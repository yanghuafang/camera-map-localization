#ifndef CAM_LOC_APPS_COMMON_ARG_PARSE_H_
#define CAM_LOC_APPS_COMMON_ARG_PARSE_H_

/// Non-throwing numeric parsing for the CLI front-ends.
///
/// std::stod and friends report a bad argument by throwing, which Google style
/// does not use. These return false instead, and reject a partial parse, so
/// `--sequence 3x` is an error rather than a silent 3.

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>

namespace cam_loc::apps {

inline bool ParseDouble(const std::string& text, double* out) {
  if (text.empty()) return false;
  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (errno != 0 || end != text.c_str() + text.size()) return false;
  *out = value;
  return true;
}

inline bool ParseFloat(const std::string& text, float* out) {
  if (text.empty()) return false;
  errno = 0;
  char* end = nullptr;
  const float value = std::strtof(text.c_str(), &end);
  if (errno != 0 || end != text.c_str() + text.size()) return false;
  *out = value;
  return true;
}

inline bool ParseInt(const std::string& text, int* out) {
  if (text.empty()) return false;
  errno = 0;
  char* end = nullptr;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (errno != 0 || end != text.c_str() + text.size()) return false;
  if (value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max()) {
    return false;
  }
  *out = static_cast<int>(value);
  return true;
}

inline bool ParseUint32(const std::string& text, uint32_t* out) {
  // strtoul accepts a leading '-' and wraps it around, which would turn
  // `--noise-seed -1` into 4294967295 rather than an error.
  if (text.empty() || text[0] == '-') return false;
  errno = 0;
  char* end = nullptr;
  const unsigned long value = std::strtoul(text.c_str(), &end, 10);
  if (errno != 0 || end != text.c_str() + text.size()) return false;
  if (value > std::numeric_limits<uint32_t>::max()) return false;
  *out = static_cast<uint32_t>(value);
  return true;
}

}  // namespace cam_loc::apps

#endif  // CAM_LOC_APPS_COMMON_ARG_PARSE_H_
