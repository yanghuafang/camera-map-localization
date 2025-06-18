/// Background thread: raw terminal stdin → step / play / quit commands.
#include "cam_loc_ros/keyboard_input.h"

#include <termios.h>
#include <unistd.h>

#include <cstdio>

namespace cam_loc_ros {

namespace {

termios g_orig_termios{};
bool g_termios_saved = false;

void RestoreTerminal() {
  if (g_termios_saved) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
    g_termios_saved = false;
  }
}

}  // namespace

KeyboardInput::KeyboardInput() = default;

KeyboardInput::~KeyboardInput() { Stop(); }

void KeyboardInput::Start() {
  // Only with a real terminal, and this is load bearing. The reader thread
  // blocks in getchar(); what makes that safe is the raw mode set below, where
  // VMIN=0 and VTIME=0 turn the read into a poll. Both need a terminal to apply
  // to. Under `ros2 launch` stdin is a pipe that never reaches EOF, raw mode
  // cannot be set, and the thread parks in getchar() forever -- so Stop()'s
  // join() never returns, the node ignores SIGTERM, and launch escalates to
  // SIGKILL ten seconds later. Closing RViz appeared to hang for exactly that
  // reason.
  if (!isatty(STDIN_FILENO)) return;
  if (running_.exchange(true)) return;

  termios raw{};
  if (tcgetattr(STDIN_FILENO, &g_orig_termios) == 0) {
    g_termios_saved = true;
    raw = g_orig_termios;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  thread_ = std::thread([this]() { Run(); });
}

void KeyboardInput::Stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) {
    thread_.join();
  }
  RestoreTerminal();
}

KeyboardInput::Command KeyboardInput::Poll() {
  const int cmd = pending_.exchange(static_cast<int>(Command::kNone));
  return static_cast<Command>(cmd);
}

void KeyboardInput::Run() {
  while (running_.load()) {
    const int c = std::getchar();
    if (c == EOF) {
      usleep(10000);
      continue;
    }
    if (c == ' ') {
      pending_.store(static_cast<int>(Command::kStep));
    } else if (c == 'r' || c == 'R') {
      pending_.store(static_cast<int>(Command::kToggleContinuous));
    } else if (c == 'q' || c == 'Q') {
      pending_.store(static_cast<int>(Command::kQuit));
      running_.store(false);
    }
  }
}

}  // namespace cam_loc_ros
