#pragma once

/// RViz marker builders for map polylines, perception, cost argmin, and trajectory paths.
///
/// All markers use the fixed `map` frame (see kMapFrame).

#include <cam_loc/core/localization_debug.hpp>
#include <cam_loc/core/projection.hpp>
#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/params.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace cam_loc_ros {

constexpr const char* kMapFrame = "map";

geometry_msgs::msg::Pose mat44ToPose(const cam_loc::Mat44& T);

visualization_msgs::msg::Marker makeDeleteAll(const std::string& ns);
visualization_msgs::msg::Marker makeLineStrip(const std::string& ns, int id, float r, float g,
                                              float b, float width = 0.15);
visualization_msgs::msg::Marker makeArrow(const std::string& ns, int id, const cam_loc::Mat44& T,
                                          float r, float g, float b, double shaft_d = 0.4,
                                          double head_d = 0.8);

/// Local map polylines as colored LINE_STRIP markers.
visualization_msgs::msg::MarkerArray buildMapMarkers(const cam_loc::kitti::MapChunk& map);

/// Perception polylines lifted to ground plane in world frame.
visualization_msgs::msg::MarkerArray buildPerceptionMarkers(
    const cam_loc::kitti::FramePerception& perception, const cam_loc::core::Projection& projection,
    const cam_loc::Mat44& T_world_cam);

/// Best pose-grid sample (arrow + sphere) from aggregated cost argmin.
visualization_msgs::msg::MarkerArray buildCostArgminMarker(
    const cam_loc::core::LocalizationDebugSnapshot& debug, const cam_loc::LocalizationResult& result);

nav_msgs::msg::Path buildPath(const std::vector<cam_loc::kitti::Pose>& poses);

nav_msgs::msg::Path buildPathFromMatrices(const std::vector<cam_loc::Mat44>& poses);

}  // namespace cam_loc_ros
