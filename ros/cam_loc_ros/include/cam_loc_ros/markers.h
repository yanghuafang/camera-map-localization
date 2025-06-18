#ifndef CAM_LOC_ROS_MARKERS_H_
#define CAM_LOC_ROS_MARKERS_H_

/// RViz marker builders for map polylines, perception, cost argmin, and
/// trajectory paths.
///
/// All markers use the fixed `map` frame (see kMapFrame).

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cam_loc/core/localization_debug.h"
#include "cam_loc/core/projection.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/types/params.h"

namespace cam_loc_ros {

constexpr const char* kMapFrame = "map";

geometry_msgs::msg::Pose Mat44ToPose(const cam_loc::Mat44& T);

visualization_msgs::msg::Marker MakeDeleteAll(const std::string& ns);
visualization_msgs::msg::Marker MakeLineStrip(const std::string& ns, int id,
                                              float r, float g, float b,
                                              float width = 0.15);
visualization_msgs::msg::Marker MakeArrow(const std::string& ns, int id,
                                          const cam_loc::Mat44& T, float r,
                                          float g, float b,
                                          double shaft_d = 0.4,
                                          double head_d = 0.8);

/// Local map polylines as colored LINE_STRIP markers.
visualization_msgs::msg::MarkerArray BuildMapMarkers(
    const cam_loc::kitti::MapChunk& map);

/// Perception polylines lifted to ground plane in world frame.
visualization_msgs::msg::MarkerArray BuildPerceptionMarkers(
    const cam_loc::kitti::FramePerception& perception,
    const cam_loc::core::Projection& projection,
    const cam_loc::Mat44& T_world_cam);

/// Best pose-grid sample (arrow + sphere) from aggregated cost argmin.
visualization_msgs::msg::MarkerArray BuildCostArgminMarker(
    const cam_loc::core::LocalizationDebugSnapshot& debug,
    const cam_loc::LocalizationResult& result);

nav_msgs::msg::Path BuildPath(const std::vector<cam_loc::kitti::Pose>& poses);

nav_msgs::msg::Path BuildPathFromMatrices(
    const std::vector<cam_loc::Mat44>& poses);

}  // namespace cam_loc_ros

#endif  // CAM_LOC_ROS_MARKERS_H_
