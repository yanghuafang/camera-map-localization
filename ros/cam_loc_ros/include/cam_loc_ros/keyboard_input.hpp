#pragma once

/// Non-blocking stdin keyboard controls for interactive sequence playback.
///
/// SPACE = step one frame, R = toggle continuous/pause, Q = quit.
/// Terminal running the node must have focus for key events.

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

  void start();
  void stop();

  /// Returns pending command and clears it (at most one per poll).
  Command poll();

 private:
  void run();

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> pending_{static_cast<int>(Command::kNone)};
};

}  // namespace cam_loc_ros
