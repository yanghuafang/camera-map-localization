#ifndef CAM_LOC_SEMANTIC_KITTI_PREPROCESS_H_
#define CAM_LOC_SEMANTIC_KITTI_PREPROCESS_H_

/// SemanticKITTI label raster → cam_loc::kitti::FramePerception.

#include <string>
#include <vector>

#include "cam_loc/kitti/types.h"
#include "cam_loc/types/status.h"

namespace cam_loc::semantic_kitti {

/// Raw SemanticKITTI class ids, as stored in the `.label` files.
///
/// These are the dataset's own ids, not the 20-class ids the training remap
/// produces — that remap folds lane markings into road, which would throw away
/// the one class the lateral constraint depends on.
///
/// What is not here is worth knowing: SemanticKITTI has **no traffic-light and
/// no crosswalk class**. Those two landmark types cannot be extracted from this
/// dataset at all, whatever the localizer would like; see docs/ARCHITECTURE.md.
enum LabelId : uint16_t {
  kRoad = 40,
  kParking = 44,
  kSidewalk = 48,
  kLaneMarking = 60,
  kTerrain = 72,
  kPole = 80,
  kTrafficSign = 81,
};

/// Tunables for label raster → polyline extraction.
struct PreprocessOptions {
  int frame = 0;
  /// Raster size. Must match the canvas the matcher rasterizes onto
  /// (LocalizationParams::image_width / image_height), or perception and map
  /// land in different pixel grids.
  int image_width = 1241;
  int image_height = 376;
  /// Scan every Nth row when tracing road markings and boundaries, and every
  /// Nth column when tracing upright landmarks.
  int scan_stride = 4;
  /// Pixels of consecutive same-class run needed to emit a polyline.
  int min_run_length = 8;
};

/// Load a 16-bit grayscale label PNG.
///
/// @return `kIoError` if the image cannot be read, `kInvalidArgument` if its
///         size differs from @p expected_width × @p expected_height.
Status LoadLabelImage16(const std::string& path, int expected_width,
                        int expected_height, std::vector<uint16_t>& out_labels);

/// Extract localization landmarks from a label raster.
///
/// Three scans, each matched to how its class sits in the image:
///
/// - **Lane markings** run across the image, so horizontal runs of class 60
///   trace them.
/// - **Road boundaries** are where the drivable surface ends, so each scanned
///   row contributes its leftmost and rightmost road pixel. Tracing runs of
///   sidewalk or terrain instead would follow the *far* side of the kerb, and
///   would depend on what happens to be beyond it.
/// - **Poles and traffic signs** stand upright, so vertical runs of classes 80
///   and 81 trace them. A horizontal scan finds a pole one or two pixels at a
///   time and rejects it as too short.
///
/// @param labels Row-major, `width * height` entries.
/// @param out    Cleared, then filled.
/// @return `kInvalidArgument` when @p labels does not match @p width ×
///         @p height.
Status LabelsToPerception(const std::vector<uint16_t>& labels, int width,
                          int height, const PreprocessOptions& opts,
                          kitti::FramePerception& out);

/// Write one frame in the perception JSON schema (see docs/KITTI_DATA.md).
Status WritePerceptionJson(const std::string& path,
                           const kitti::FramePerception& perception);

}  // namespace cam_loc::semantic_kitti

#endif  // CAM_LOC_SEMANTIC_KITTI_PREPROCESS_H_
