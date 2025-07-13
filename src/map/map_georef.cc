// WGS84 ↔ KITTI world: local ENU tangent plane plus optional horizontal yaw
// alignment.

#include "cam_loc/map/map_georef.h"

#include <cmath>
#include <fstream>

#include <nlohmann/json.hpp>

#include "cam_loc/core/frames.h"

namespace cam_loc::map {

namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

bool MapGeoref::IsValid() const {
  return std::abs(origin_lat_deg) <= 90.0 && std::abs(origin_lon_deg) <= 180.0;
}

MapGeoref MapGeoref::FromFirstPoseHeading(double lat_deg, double lon_deg,
                                          double heading_rad, double alt_m) {
  MapGeoref g;
  g.origin_lat_deg = lat_deg;
  g.origin_lon_deg = lon_deg;
  g.origin_alt_m = alt_m;
  g.world_yaw_rad = heading_rad;
  return g;
}

Status MapGeoref::ParseFromJsonString(const std::string& json_text) {
  // allow_exceptions=false: malformed text yields a discarded value rather
  // than a throw. Every read below is already guarded by contains(), so nothing
  // else here can throw either.
  const auto j =
      nlohmann::json::parse(json_text, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) return Status::kInvalidArgument;
  if (j.contains("origin_lat_deg"))
    origin_lat_deg = j["origin_lat_deg"].get<double>();
  if (j.contains("origin_lon_deg"))
    origin_lon_deg = j["origin_lon_deg"].get<double>();
  if (j.contains("origin_alt_m"))
    origin_alt_m = j["origin_alt_m"].get<double>();
  if (j.contains("world_yaw_deg")) {
    world_yaw_rad = j["world_yaw_deg"].get<double>() * kDegToRad;
  } else if (j.contains("world_yaw_rad")) {
    world_yaw_rad = j["world_yaw_rad"].get<double>();
  }
  return IsValid() ? Status::kOk : Status::kInvalidArgument;
}

Status MapGeoref::LoadFromJsonFile(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) return Status::kIoError;
  std::string text((std::istreambuf_iterator<char>(in)),
                   std::istreambuf_iterator<char>());
  return ParseFromJsonString(text);
}

Vec3 MapGeoref::Wgs84ToWorld(double lat_deg, double lon_deg,
                             double alt_m) const {
  // Small-area flat-earth ENU about the origin.
  const double lat0 = origin_lat_deg * kDegToRad;
  const double dlat = (lat_deg - origin_lat_deg) * kDegToRad;
  const double dlon = (lon_deg - origin_lon_deg) * kDegToRad;

  const double north = dlat * kEarthRadiusM;
  const double east = dlon * kEarthRadiusM * std::cos(lat0);
  const double up = alt_m - origin_alt_m;

  // Rotate ENU so the map's forward axis lines up with the sequence heading.
  const double c = std::cos(world_yaw_rad);
  const double s = std::sin(world_yaw_rad);
  const double right = c * east + s * north;
  const double forward = -s * east + c * north;

  // Then into cam0, which is the frame the poses and the pose grid live in:
  // X right, Y down, Z forward. Returning ENU here put every OSM polyline in a
  // different frame from the trajectory it was meant to be matched against.
  return core::Frames::ToCam0(Vec3(forward, -right, up));
}

}  // namespace cam_loc::map
