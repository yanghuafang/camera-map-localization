#ifndef CAM_LOC_ROS_KEYBOARD_INPUT_H_
#define CAM_LOC_ROS_KEYBOARD_INPUT_H_

/// Non-blocking stdin keyboard controls for interactive sequence playback.
///
/// SPACE = step one frame, R = toggle continuous/pause, Q = quit.
/// Terminal running the node must have focus for key events.
///
/// Inert without a terminal, which is the case under `ros2 launch`. See
/// Start() for why that is a correctness requirement and not just tidiness.

#include <atomic>
#include <thread>

namespace cam_loc_ros {

/// Reads single-key commands from stdin (terminal must be focused).
/// SPACE = step one frame, R = toggle continuous/pause, Q = quit.
class KeyboardInput {
 public:
  enum class Command { kNone, kStep, kToggleContinuous, kQuit };

  KeyboardInput();
  ~KeyboardInput();

  KeyboardInput(const KeyboardInput&) = delete;
  KeyboardInput& operator=(const KeyboardInput&) = delete;

  void Start();
  void Stop();

  /// Whether Start() actually attached to a terminal. False under `ros2
  /// launch`, where there is no terminal to read and the node's key bindings
  /// therefore do nothing.
  bool attached() const { return running_.load(); }

  /// Returns pending command and clears it (at most one per Poll() call).
  Command Poll();

 private:
  void Run();

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> pending_{static_cast<int>(Command::kNone)};
};

}  // namespace cam_loc_ros

#endif  // CAM_LOC_ROS_KEYBOARD_INPUT_H_
