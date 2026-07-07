/// RViz marker construction for cam_loc_viz_node publishers.
#include <cam_loc_ros/markers.hpp>

#include <Eigen/Geometry>

namespace cam_loc_ros {

namespace {

geometry_msgs::msg::Point eigenToPoint(const cam_loc::Vec3& p) {
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

void setColor(visualization_msgs::msg::Marker& m, float r, float g, float b, float a = 1.f) {
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
}

}  // namespace

geometry_msgs::msg::Pose mat44ToPose(const cam_loc::Mat44& T) {
  geometry_msgs::msg::Pose out;
  const Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  out.position.x = T(0, 3);
  out.position.y = T(1, 3);
  out.position.z = T(2, 3);
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}

visualization_msgs::msg::Marker makeDeleteAll(const std::string& ns) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = kMapFrame;
  m.ns = ns;
  m.id = 0;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  return m;
}

visualization_msgs::msg::Marker makeLineStrip(const std::string& ns, int id, float r, float g,
                                              float b, float width) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = kMapFrame;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = width;
  setColor(m, r, g, b);
  m.pose.orientation.w = 1.0;
  return m;
}

visualization_msgs::msg::Marker makeArrow(const std::string& ns, int id, const cam_loc::Mat44& T,
                                          float r, float g, float b, double shaft_d,
                                          double head_d) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = kMapFrame;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose = mat44ToPose(T);
  m.scale.x = shaft_d;
  m.scale.y = head_d * 0.35;
  m.scale.z = head_d * 0.35;
  setColor(m, r, g, b);
  return m;
}

visualization_msgs::msg::MarkerArray buildMapMarkers(const cam_loc::kitti::MapChunk& map) {
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(makeDeleteAll("map"));
  int id = 1;
  for (const auto& pl : map.polylines) {
    if (pl.points.size() < 2) continue;
    float r = 1.f;
    float g = 0.85f;
    float b = 0.2f;
    if (pl.type == cam_loc::kitti::PolylineType::kRoadEdge) {
      r = 1.f;
      g = 0.55f;
      b = 0.1f;
    } else if (pl.type == cam_loc::kitti::PolylineType::kLaneDashed) {
      r = 0.9f;
      g = 0.9f;
      b = 0.3f;
    }
    auto m = makeLineStrip("map", id++, r, g, b, 0.12);
    for (const auto& p : pl.points) {
      m.points.push_back(eigenToPoint(p));
    }
    arr.markers.push_back(std::move(m));
  }
  return arr;
}

visualization_msgs::msg::MarkerArray buildPerceptionMarkers(
    const cam_loc::kitti::FramePerception& perception, const cam_loc::core::Projection& projection,
    const cam_loc::Mat44& T_world_cam) {
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(makeDeleteAll("perception"));

  auto addPolylines = [&](const std::vector<cam_loc::kitti::Polyline2D>& polylines, float r,
                          float g, float b, int& id) {
    for (const auto& pl : polylines) {
      if (pl.points.size() < 2) continue;
      auto m = makeLineStrip("perception", id++, r, g, b, 0.18);
      for (const auto& uv : pl.points) {
        cam_loc::Vec3 p_world;
        if (projection.imageToWorldGround(T_world_cam, uv, p_world) != cam_loc::Status::kOk) {
          continue;
        }
        m.points.push_back(eigenToPoint(p_world));
      }
      if (m.points.size() >= 2) {
        arr.markers.push_back(std::move(m));
      }
    }
  };

  int id = 1;
  addPolylines(perception.lane_lines, 0.2f, 0.95f, 0.35f, id);
  addPolylines(perception.road_boundaries, 0.2f, 0.85f, 1.f, id);
  return arr;
}

visualization_msgs::msg::MarkerArray buildCostArgminMarker(
    const cam_loc::core::LocalizationDebugSnapshot& debug, const cam_loc::LocalizationResult& result) {
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(makeDeleteAll("cost"));
  if (!debug.valid) return arr;

  const cam_loc::Vec3 offset = result.best_sample_xyyaw;
  const cam_loc::Mat44 T_sample =
      debug.T_world_plane *
      cam_loc::core::Projection::offsetToTransform(offset.x(), offset.y(), offset.z());

  arr.markers.push_back(makeArrow("cost", 1, T_sample, 1.f, 0.2f, 1.f, 0.6, 1.0));

  auto sphere = makeLineStrip("cost", 2, 1.f, 0.f, 1.f, 0.25);
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.pose = mat44ToPose(T_sample);
  sphere.scale.x = 0.5;
  sphere.scale.y = 0.5;
  sphere.scale.z = 0.5;
  arr.markers.push_back(std::move(sphere));
  return arr;
}

nav_msgs::msg::Path buildPath(const std::vector<cam_loc::kitti::Pose>& poses) {
  nav_msgs::msg::Path path;
  path.header.frame_id = kMapFrame;
  for (const auto& p : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = kMapFrame;
    ps.pose = mat44ToPose(p.T_world_cam0);
    path.poses.push_back(ps);
  }
  return path;
}

nav_msgs::msg::Path buildPathFromMatrices(const std::vector<cam_loc::Mat44>& poses) {
  nav_msgs::msg::Path path;
  path.header.frame_id = kMapFrame;
  for (const auto& T : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = kMapFrame;
    ps.pose = mat44ToPose(T);
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace cam_loc_ros
