/// RViz marker construction for cam_loc_viz_node publishers.
#include "cam_loc_ros/markers.h"

#include <Eigen/Geometry>

#include "cam_loc/core/frames.h"

namespace cam_loc_ros {

namespace {

// Every pose, map point and world coordinate in this project is in KITTI's
// cam0: X right, Y down, Z forward. RViz draws its fixed frame by the ROS
// convention, X forward, Y left, Z up. Published unconverted, cam0's forward
// axis is drawn as up -- which stands the road on end and puts the lane
// markings in the sky.
//
// core::Frames::ToVehicle is precisely that change of basis, so this reuses the
// project's one convention rather than introducing a second. Both funnels below
// apply it, which is why nothing else in this file has to think about frames.
geometry_msgs::msg::Point EigenToPoint(const cam_loc::Vec3& p_cam0) {
  const cam_loc::Vec3 p = cam_loc::core::Frames::ToVehicle(p_cam0);
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

void SetColor(visualization_msgs::msg::Marker& m, float r, float g, float b,
              float a = 1.f) {
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
}

}  // namespace

geometry_msgs::msg::Pose Mat44ToPose(const cam_loc::Mat44& T_cam0) {
  // A change of basis on both sides, not just on the translation: the world is
  // re-expressed in the vehicle convention and so is the body, so an arrow
  // marker points along travel instead of along the optical axis.
  const Eigen::Matrix3d R_vehicle_cam0 =
      cam_loc::core::Frames::RotCam0Vehicle().transpose();
  const cam_loc::Vec3 t =
      cam_loc::core::Frames::ToVehicle(T_cam0.block<3, 1>(0, 3));
  const Eigen::Matrix3d R =
      R_vehicle_cam0 * T_cam0.block<3, 3>(0, 0) * R_vehicle_cam0.transpose();

  geometry_msgs::msg::Pose out;
  const Eigen::Quaterniond q(R);
  out.position.x = t.x();
  out.position.y = t.y();
  out.position.z = t.z();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}

visualization_msgs::msg::Marker MakeDeleteAll(const std::string& ns) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = kMapFrame;
  m.ns = ns;
  m.id = 0;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  return m;
}

visualization_msgs::msg::Marker MakeLineStrip(const std::string& ns, int id,
                                              float r, float g, float b,
                                              float width) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = kMapFrame;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = width;
  SetColor(m, r, g, b);
  m.pose.orientation.w = 1.0;
  return m;
}

visualization_msgs::msg::Marker MakeArrow(const std::string& ns, int id,
                                          const cam_loc::Mat44& T, float r,
                                          float g, float b, double shaft_d,
                                          double head_d) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = kMapFrame;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose = Mat44ToPose(T);
  m.scale.x = shaft_d;
  m.scale.y = head_d * 0.35;
  m.scale.z = head_d * 0.35;
  SetColor(m, r, g, b);
  return m;
}

visualization_msgs::msg::MarkerArray BuildMapMarkers(
    const cam_loc::kitti::MapChunk& map) {
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(MakeDeleteAll("map"));
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
    auto m = MakeLineStrip("map", id++, r, g, b, 0.12);
    for (const auto& p : pl.points) {
      m.points.push_back(EigenToPoint(p));
    }
    arr.markers.push_back(std::move(m));
  }
  return arr;
}

visualization_msgs::msg::MarkerArray BuildPerceptionMarkers(
    const cam_loc::kitti::FramePerception& perception,
    const cam_loc::core::Projection& projection,
    const cam_loc::Mat44& T_world_cam) {
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(MakeDeleteAll("perception"));

  int id = 1;
  for (const auto& pl : perception.features) {
    // Ground classes only. ImageToWorldGround intersects the pixel ray with the
    // road plane, which is where a lane marking or a curb lies; a pole or a
    // sign extends away from it, so the same projection would scatter their
    // upper points out towards the horizon.
    float r = 0.2f;
    float g = 0.95f;
    float b = 0.35f;
    switch (pl.type) {
      case cam_loc::kitti::PolylineType::kLaneSolid:
      case cam_loc::kitti::PolylineType::kLaneDashed:
        break;
      case cam_loc::kitti::PolylineType::kRoadEdge:
        r = 0.2f;
        g = 0.85f;
        b = 1.f;
        break;
      case cam_loc::kitti::PolylineType::kPole:
      case cam_loc::kitti::PolylineType::kSign:
      case cam_loc::kitti::PolylineType::kUnknown:
        continue;
    }
    if (pl.points.size() < 2) continue;

    auto m = MakeLineStrip("perception", id++, r, g, b, 0.18);
    for (const auto& uv : pl.points) {
      cam_loc::Vec3 p_world;
      if (projection.ImageToWorldGround(T_world_cam, uv, p_world) !=
          cam_loc::Status::kOk) {
        continue;
      }
      m.points.push_back(EigenToPoint(p_world));
    }
    if (m.points.size() >= 2) {
      arr.markers.push_back(std::move(m));
    }
  }
  return arr;
}

visualization_msgs::msg::MarkerArray BuildCostArgminMarker(
    const cam_loc::core::LocalizationDebugSnapshot& debug,
    const cam_loc::LocalizationResult& result) {
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(MakeDeleteAll("cost"));
  if (!debug.valid) return arr;

  const cam_loc::Vec3 offset = result.best_sample_xyyaw;
  const cam_loc::Mat44 T_sample =
      debug.T_world_plane * cam_loc::core::Frames::OffsetToCam0Transform(
                                offset.x(), offset.y(), offset.z());

  arr.markers.push_back(
      MakeArrow("cost", 1, T_sample, 1.f, 0.2f, 1.f, 0.6, 1.0));

  auto sphere = MakeLineStrip("cost", 2, 1.f, 0.f, 1.f, 0.25);
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.pose = Mat44ToPose(T_sample);
  sphere.scale.x = 0.5;
  sphere.scale.y = 0.5;
  sphere.scale.z = 0.5;
  arr.markers.push_back(std::move(sphere));
  return arr;
}

nav_msgs::msg::Path BuildPath(const std::vector<cam_loc::kitti::Pose>& poses) {
  nav_msgs::msg::Path path;
  path.header.frame_id = kMapFrame;
  for (const auto& p : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = kMapFrame;
    ps.pose = Mat44ToPose(p.T_world_cam0);
    path.poses.push_back(ps);
  }
  return path;
}

nav_msgs::msg::Path BuildPathFromMatrices(
    const std::vector<cam_loc::Mat44>& poses) {
  nav_msgs::msg::Path path;
  path.header.frame_id = kMapFrame;
  for (const auto& T : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = kMapFrame;
    ps.pose = Mat44ToPose(T);
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace cam_loc_ros
