/// ROS2 playback node: step/continuous KITTI sequence through LocalizationEngine.
///
/// Publishes map/perception/cost markers, GT/estimate paths, camera image, and status.
/// Playback flow mirrors run_sequence: buildEgomotion → resolvePerception → processFrame.
#include <cam_loc/core/localization_engine.hpp>
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/map/map_loader_util.hpp>
#include <cam_loc/perception/resolve.hpp>

#include <cam_loc_ros/keyboard_input.hpp>
#include <cam_loc_ros/markers.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <stb_image.h>

namespace {

cam_loc::perception::PerceptionSource parsePerceptionMode(const std::string& s) {
  if (s == "auto") return cam_loc::perception::PerceptionSource::kAuto;
  if (s == "file") return cam_loc::perception::PerceptionSource::kFile;
  if (s == "oracle") return cam_loc::perception::PerceptionSource::kOracle;
  if (s == "noisy") return cam_loc::perception::PerceptionSource::kNoisy;
  throw std::runtime_error("Unknown perception_mode: " + s);
}

sensor_msgs::msg::Image loadGrayImageMsg(const std::string& path, const rclcpp::Time& stamp) {
  sensor_msgs::msg::Image msg;
  int w = 0;
  int h = 0;
  int comp = 0;
  unsigned char* data = stbi_load(path.c_str(), &w, &h, &comp, 1);
  if (!data) {
    return msg;
  }
  msg.header.stamp = stamp;
  msg.header.frame_id = cam_loc_ros::kMapFrame;
  msg.height = static_cast<uint32_t>(h);
  msg.width = static_cast<uint32_t>(w);
  msg.encoding = "mono8";
  msg.is_bigendian = false;
  msg.step = static_cast<uint32_t>(w);
  msg.data.assign(data, data + static_cast<size_t>(w * h));
  stbi_image_free(data);
  return msg;
}

}  // namespace

class CamLocVizNode : public rclcpp::Node {
 public:
  CamLocVizNode() : Node("cam_loc_viz") {
    kitti_root_ = declare_parameter<std::string>("kitti_root", ".");
    perception_root_ = declare_parameter<std::string>("perception_root", "");
    map_path_ = declare_parameter<std::string>("map_path", "");
    georef_path_ = declare_parameter<std::string>("georef_path", "");
    perception_mode_ = declare_parameter<std::string>("perception_mode", "auto");
    sequence_ = declare_parameter<int>("sequence", 0);
    start_frame_ = declare_parameter<int>("start_frame", 0);
    max_frames_ = declare_parameter<int>("max_frames", -1);
    use_gt_plane_ = declare_parameter<bool>("use_gt_plane", false);
    use_cuda_ = declare_parameter<bool>("use_cuda", false);
    playback_hz_ = declare_parameter<double>("playback_hz", 5.0);
    map_align_yaw_ = declare_parameter<bool>("map_align_yaw", false);
    map_origin_lat_ = declare_parameter<double>("map_origin_lat", 0.0);
    map_origin_lon_ = declare_parameter<double>("map_origin_lon", 0.0);
    map_origin_set_ = declare_parameter<bool>("map_origin_set", false);

    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("map_markers", 10);
    perception_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("perception_markers", 10);
    cost_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("cost_markers", 10);
    gt_path_pub_ = create_publisher<nav_msgs::msg::Path>("gt_path", 10);
    est_path_pub_ = create_publisher<nav_msgs::msg::Path>("estimate_path", 10);
    gt_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gt_pose", 10);
    est_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("estimate_pose", 10);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);

    if (!loadDataset()) {
      throw std::runtime_error("Failed to load KITTI dataset");
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu poses for sequence %02d", poses_.size(), sequence_);
    RCLCPP_INFO(get_logger(),
                "Keyboard controls (focus this terminal):");
    RCLCPP_INFO(get_logger(), "  SPACE  step one frame");
    RCLCPP_INFO(get_logger(), "  R      toggle continuous play / pause");
    RCLCPP_INFO(get_logger(), "  Q      quit");

    keyboard_.start();
    current_frame_ = std::max(0, start_frame_);
    const int end = max_frames_ < 0 ? static_cast<int>(poses_.size())
                                    : std::min(max_frames_, static_cast<int>(poses_.size()));
    end_frame_ = end;

    cam_loc::LocalizationParams params;
    params.use_gt_sampling_plane = use_gt_plane_;
    params.use_cuda = use_cuda_;
    map_query_radius_m_ = params.map_query_radius_m;
    engine_ = std::make_unique<cam_loc::core::LocalizationEngine>(params);
    engine_->setMapLoader(map_loader_);
    engine_->setCalibration(calib_);
    projection_ = std::make_unique<cam_loc::core::Projection>(calib_);

    gt_path_msg_ = cam_loc_ros::buildPath(poses_);
    est_path_msg_.header.frame_id = cam_loc_ros::kMapFrame;
    estimate_poses_.reserve(poses_.size());

    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&CamLocVizNode::onTimer, this));
  }

  ~CamLocVizNode() override { keyboard_.stop(); }

 private:
  bool loadDataset() {
    const std::string poses_path =
        cam_loc::kitti::resolvePosesPath(kitti_root_, sequence_);
    if (cam_loc::kitti::loadPosesFile(poses_path, poses_) != cam_loc::Status::kOk) {
      RCLCPP_ERROR(get_logger(), "Cannot load poses: %s", poses_path.c_str());
      return false;
    }
    const std::string calib_path = cam_loc::kitti::resolveCalibPath(kitti_root_, sequence_);
    if (cam_loc::kitti::parseCalibrationFile(calib_path, calib_) != cam_loc::Status::kOk) {
      RCLCPP_ERROR(get_logger(), "Cannot load calib: %s", calib_path.c_str());
      return false;
    }

    cam_loc::map::MapLoadOptions map_opt;
    map_opt.map_path = map_path_;
    map_opt.georef_path = georef_path_;
    map_opt.poses = &poses_;
    map_opt.align_yaw_to_first_pose = map_align_yaw_;
    if (map_origin_set_) {
      map_opt.georef.origin_lat_deg = map_origin_lat_;
      map_opt.georef.origin_lon_deg = map_origin_lon_;
    }
    return cam_loc::map::createMapLoader(map_opt, map_loader_) == cam_loc::Status::kOk;
  }

  void publishStatus(const std::string& text) {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  void processUntil(int target_frame) {
    // Catch up localization to target_frame; debug snapshot captured on that frame only.
    while (last_processed_ < target_frame && last_processed_ + 1 < end_frame_) {
      const int f = last_processed_ + 1;
      cam_loc::kitti::Egomotion ego;
      if (cam_loc::kitti::buildEgomotion(poses_, f, ego) != cam_loc::Status::kOk) {
        RCLCPP_ERROR(get_logger(), "Egomotion failed at frame %d", f);
        return;
      }

      cam_loc::kitti::FramePerception perception;
      cam_loc::perception::PerceptionResolveInfo pinfo;
      const auto& T_gt = poses_[static_cast<size_t>(f)].T_world_cam0;
      const auto pmode = parsePerceptionMode(perception_mode_);
      if (cam_loc::perception::resolvePerception(
              pmode, perception_root_, sequence_, f, *map_loader_, map_query_radius_m_,
              *projection_, T_gt, {}, 1, perception, pinfo) != cam_loc::Status::kOk &&
          pmode == cam_loc::perception::PerceptionSource::kOracle) {
        RCLCPP_WARN(get_logger(), "Oracle perception failed at frame %d", f);
      }

      engine_->setDebugCapture(f == target_frame);
      engine_->processFrame(ego, perception);
      estimate_poses_.push_back(engine_->result().T_world_rig);
      last_processed_ = f;
    }
  }

  void publishFrame(int frame) {
    if (frame < 0 || frame >= end_frame_) return;

    processUntil(frame);

    const rclcpp::Time stamp = now();
    auto stampMarkers = [&](visualization_msgs::msg::MarkerArray& arr) {
      for (auto& m : arr.markers) {
        m.header.stamp = stamp;
      }
    };

    if (!engine_->debugSnapshot().valid) {
      engine_->setDebugCapture(true);
      processUntil(frame);
    }

    const auto& dbg = engine_->debugSnapshot();
    auto map_markers = cam_loc_ros::buildMapMarkers(dbg.local_map);
    stampMarkers(map_markers);
    map_pub_->publish(map_markers);

    auto perception_markers =
        cam_loc_ros::buildPerceptionMarkers(dbg.perception, *projection_, dbg.T_world_plane);
    stampMarkers(perception_markers);
    perception_pub_->publish(perception_markers);

    auto cost_markers = cam_loc_ros::buildCostArgminMarker(dbg, engine_->result());
    stampMarkers(cost_markers);
    cost_pub_->publish(cost_markers);

    gt_path_msg_.header.stamp = stamp;
    for (size_t i = 0; i < gt_path_msg_.poses.size(); ++i) {
      gt_path_msg_.poses[i].header.stamp = stamp;
    }
    gt_path_pub_->publish(gt_path_msg_);

    est_path_msg_ = cam_loc_ros::buildPathFromMatrices(estimate_poses_);
    est_path_msg_.header.stamp = stamp;
    for (auto& ps : est_path_msg_.poses) {
      ps.header.stamp = stamp;
    }
    est_path_pub_->publish(est_path_msg_);

    geometry_msgs::msg::PoseStamped gt_pose;
    gt_pose.header.stamp = stamp;
    gt_pose.header.frame_id = cam_loc_ros::kMapFrame;
    gt_pose.pose = cam_loc_ros::mat44ToPose(poses_[static_cast<size_t>(frame)].T_world_cam0);
    gt_pose_pub_->publish(gt_pose);

    geometry_msgs::msg::PoseStamped est_pose;
    est_pose.header.stamp = stamp;
    est_pose.header.frame_id = cam_loc_ros::kMapFrame;
    est_pose.pose = cam_loc_ros::mat44ToPose(engine_->result().T_world_rig);
    est_pose_pub_->publish(est_pose);

    const std::string image_path =
        cam_loc::kitti::resolveImagePath(kitti_root_, sequence_, frame);
    if (std::filesystem::is_regular_file(image_path)) {
      auto img = loadGrayImageMsg(image_path, stamp);
      if (!img.data.empty()) {
        image_pub_->publish(img);
      }
    }

    std::ostringstream oss;
    oss << "frame " << frame << "/" << (end_frame_ - 1)
        << " match=" << (engine_->result().sampling_measurement_applied ? "yes" : "no")
        << " cost=" << engine_->result().aggregate_min_cost;
    publishStatus(oss.str());
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  void onTimer() {
    const auto cmd = keyboard_.poll();
    if (cmd == cam_loc_ros::KeyboardInput::Command::kQuit) {
      RCLCPP_INFO(get_logger(), "Quit requested");
      rclcpp::shutdown();
      return;
    }
    if (cmd == cam_loc_ros::KeyboardInput::Command::kToggleContinuous) {
      continuous_ = !continuous_;
      RCLCPP_INFO(get_logger(), continuous_ ? "Continuous playback ON" : "Continuous playback PAUSED");
    }
    if (cmd == cam_loc_ros::KeyboardInput::Command::kStep) {
      continuous_ = false;
      step_requested_ = true;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    bool advance = false;
    if (step_requested_) {
      advance = true;
      step_requested_ = false;
    } else if (continuous_) {
      const double hz = std::max(playback_hz_, 0.1);
      const auto period = std::chrono::duration<double>(1.0 / hz);
      if (now_tp - last_advance_tp_ >= period) {
        advance = true;
      }
    }

    if (!advance) return;

    if (current_frame_ >= end_frame_) {
      if (continuous_) {
        continuous_ = false;
        RCLCPP_INFO(get_logger(), "Reached end of sequence — paused");
      }
      return;
    }

    publishFrame(current_frame_);
    ++current_frame_;
    last_advance_tp_ = now_tp;
  }

  std::string kitti_root_;
  std::string perception_root_;
  std::string map_path_;
  std::string georef_path_;
  std::string perception_mode_;
  int sequence_ = 0;
  int start_frame_ = 0;
  int max_frames_ = -1;
  bool use_gt_plane_ = false;
  bool use_cuda_ = false;
  double playback_hz_ = 5.0;
  bool map_align_yaw_ = false;
  double map_origin_lat_ = 0.0;
  double map_origin_lon_ = 0.0;
  bool map_origin_set_ = false;
  double map_query_radius_m_ = 50.0;

  std::vector<cam_loc::kitti::Pose> poses_;
  cam_loc::kitti::Calibration calib_;
  std::shared_ptr<cam_loc::map::IMapLoader> map_loader_;
  std::unique_ptr<cam_loc::core::LocalizationEngine> engine_;
  std::unique_ptr<cam_loc::core::Projection> projection_;

  cam_loc_ros::KeyboardInput keyboard_;
  bool continuous_ = false;
  bool step_requested_ = true;
  int current_frame_ = 0;
  int end_frame_ = 0;
  int last_processed_ = -1;

  nav_msgs::msg::Path gt_path_msg_;
  nav_msgs::msg::Path est_path_msg_;
  std::vector<cam_loc::Mat44> estimate_poses_;

  std::chrono::steady_clock::time_point last_advance_tp_{};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr perception_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cost_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gt_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr est_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr est_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CamLocVizNode>());
  } catch (const std::exception& ex) {
    fprintf(stderr, "cam_loc_viz_node failed: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
