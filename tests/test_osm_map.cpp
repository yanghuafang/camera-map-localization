// Unit tests for OSM/JSON map loaders and georef.
#include <cam_loc/map/map_georef.hpp>
#include <cam_loc/map/map_loader_util.hpp>
#include <cam_loc/map/osm_map_loader.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

TEST(OsmMapLoaderTest, LoadsJsonPolylines) {
  cam_loc::map::OsmMapLoader map;
  const std::string path = std::string(TEST_DATA_DIR) + "/map_minimal.json";
  ASSERT_EQ(map.loadFromJsonFile(path), cam_loc::Status::kOk);
  EXPECT_EQ(map.map().polylines.size(), 3u);

  cam_loc::Mat44 T = cam_loc::Mat44::Identity();
  T(0, 3) = 5.0;
  cam_loc::kitti::MapChunk local;
  ASSERT_EQ(map.queryLocalMap(T, 15.0, local), cam_loc::Status::kOk);
  EXPECT_GE(local.polylines.size(), 1u);
}

TEST(OsmMapLoaderTest, LoadsNativeOsmWithGeoref) {
  cam_loc::map::OsmMapLoader map;
  cam_loc::map::MapGeoref georef;
  georef.origin_lat_deg = 49.0;
  georef.origin_lon_deg = 8.0;
  map.setGeoref(georef);

  const std::string path = std::string(TEST_DATA_DIR) + "/map_minimal.osm";
  ASSERT_EQ(map.loadFromOsmFile(path), cam_loc::Status::kOk);
  EXPECT_GE(map.map().polylines.size(), 1u);

  const auto& main_road = map.map().polylines.front();
  EXPECT_EQ(main_road.type, cam_loc::kitti::PolylineType::kLaneSolid);
  EXPECT_GE(main_road.points.size(), 2u);
  EXPECT_NEAR(main_road.points.front().x(), 0.0, 1.0);
  EXPECT_NEAR(main_road.points.front().y(), 0.0, 1.0);
  EXPECT_GT(main_road.points.back().x(), 100.0);
}

TEST(OsmMapLoaderTest, LoadFromFileDetectsExtension) {
  cam_loc::map::OsmMapLoader map;
  cam_loc::map::MapGeoref georef;
  georef.origin_lat_deg = 49.0;
  georef.origin_lon_deg = 8.0;
  map.setGeoref(georef);

  const std::string osm_path = std::string(TEST_DATA_DIR) + "/map_minimal.osm";
  ASSERT_EQ(map.loadFromFile(osm_path), cam_loc::Status::kOk);
  EXPECT_FALSE(map.map().polylines.empty());
}

TEST(MapGeorefTest, Wgs84ToWorldEastAxis) {
  cam_loc::map::MapGeoref georef;
  georef.origin_lat_deg = 49.0;
  georef.origin_lon_deg = 8.0;
  const cam_loc::Vec3 p = georef.wgs84ToWorld(49.0, 8.001, 0.0);
  EXPECT_NEAR(p.x(), 111.0 * std::cos(49.0 * M_PI / 180.0), 5.0);
  EXPECT_NEAR(p.y(), 0.0, 1e-3);
}

TEST(MapLoaderUtilTest, CorridorWhenPathEmpty) {
  std::vector<cam_loc::kitti::Pose> poses(3);
  for (size_t i = 0; i < poses.size(); ++i) {
    poses[i].T_world_cam0 = cam_loc::Mat44::Identity();
    poses[i].T_world_cam0(0, 3) = static_cast<double>(i) * 5.0;
  }

  cam_loc::map::MapLoadOptions opt;
  opt.poses = &poses;
  std::shared_ptr<cam_loc::map::IMapLoader> loader;
  ASSERT_EQ(cam_loc::map::createMapLoader(opt, loader), cam_loc::Status::kOk);
  ASSERT_NE(loader, nullptr);

  cam_loc::kitti::MapChunk local;
  cam_loc::Mat44 T = cam_loc::Mat44::Identity();
  T(0, 3) = 5.0;
  ASSERT_EQ(loader->queryLocalMap(T, 20.0, local), cam_loc::Status::kOk);
  EXPECT_GE(local.polylines.size(), 1u);
}

TEST(MapLoaderUtilTest, JsonMapViaFactory) {
  std::vector<cam_loc::kitti::Pose> poses(2);
  poses[0].T_world_cam0 = cam_loc::Mat44::Identity();
  poses[1].T_world_cam0 = cam_loc::Mat44::Identity();
  poses[1].T_world_cam0(0, 3) = 10.0;

  cam_loc::map::MapLoadOptions opt;
  opt.map_path = std::string(TEST_DATA_DIR) + "/map_minimal.json";
  opt.poses = &poses;
  std::shared_ptr<cam_loc::map::IMapLoader> loader;
  ASSERT_EQ(cam_loc::map::createMapLoader(opt, loader), cam_loc::Status::kOk);
  ASSERT_NE(loader, nullptr);
}

TEST(MapLoaderUtilTest, OsmMapViaFactoryWithGeorefFile) {
  std::vector<cam_loc::kitti::Pose> poses(2);
  poses[0].T_world_cam0 = cam_loc::Mat44::Identity();
  poses[1].T_world_cam0 = cam_loc::Mat44::Identity();
  poses[1].T_world_cam0(0, 3) = 10.0;

  cam_loc::map::MapLoadOptions opt;
  opt.map_path = std::string(TEST_DATA_DIR) + "/map_minimal.osm";
  opt.georef_path = std::string(TEST_DATA_DIR) + "/map_georef.json";
  opt.poses = &poses;
  std::shared_ptr<cam_loc::map::IMapLoader> loader;
  ASSERT_EQ(cam_loc::map::createMapLoader(opt, loader), cam_loc::Status::kOk);
  ASSERT_NE(loader, nullptr);
}
