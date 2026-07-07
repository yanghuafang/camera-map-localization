// WGS84 ↔ KITTI world: local ENU tangent plane plus optional horizontal yaw alignment.

#include <cam_loc/map/map_georef.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>

namespace cam_loc::map {

namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

bool MapGeoref::isValid() const {
  return std::abs(origin_lat_deg) <= 90.0 && std::abs(origin_lon_deg) <= 180.0;
}

MapGeoref MapGeoref::fromFirstPoseHeading(double lat_deg, double lon_deg, double heading_rad,
                                        double alt_m) {
  MapGeoref g;
  g.origin_lat_deg = lat_deg;
  g.origin_lon_deg = lon_deg;
  g.origin_alt_m = alt_m;
  g.world_yaw_rad = heading_rad;
  return g;
}

Status MapGeoref::parseFromJsonString(const std::string& json_text) {
  try {
    const auto j = nlohmann::json::parse(json_text);
    if (j.contains("origin_lat_deg")) origin_lat_deg = j["origin_lat_deg"].get<double>();
    if (j.contains("origin_lon_deg")) origin_lon_deg = j["origin_lon_deg"].get<double>();
    if (j.contains("origin_alt_m")) origin_alt_m = j["origin_alt_m"].get<double>();
    if (j.contains("world_yaw_deg")) {
      world_yaw_rad = j["world_yaw_deg"].get<double>() * kDegToRad;
    } else if (j.contains("world_yaw_rad")) {
      world_yaw_rad = j["world_yaw_rad"].get<double>();
    }
    return isValid() ? Status::kOk : Status::kInvalidArgument;
  } catch (const nlohmann::json::exception&) {
    return Status::kInvalidArgument;
  }
}

Status MapGeoref::loadFromJsonFile(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) return Status::kIoError;
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return parseFromJsonString(text);
}

Vec3 MapGeoref::wgs84ToWorld(double lat_deg, double lon_deg, double alt_m) const {
  // Small-area flat-earth ENU, then rotate (east, north) into KITTI (x, y).
  const double lat0 = origin_lat_deg * kDegToRad;
  const double lat = lat_deg * kDegToRad;
  const double dlat = lat - lat0;
  const double dlon = (lon_deg - origin_lon_deg) * kDegToRad;

  const double north = dlat * kEarthRadiusM;
  const double east = dlon * kEarthRadiusM * std::cos(lat0);
  const double up = alt_m - origin_alt_m;

  const double c = std::cos(world_yaw_rad);
  const double s = std::sin(world_yaw_rad);
  const double x = c * east + s * north;
  const double y = -s * east + c * north;
  return Vec3(x, y, up);
}

}  // namespace cam_loc::map
