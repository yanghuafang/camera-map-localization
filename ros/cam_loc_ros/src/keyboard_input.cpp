/// Background thread: raw terminal stdin → step / play / quit commands.
#include <cam_loc_ros/keyboard_input.hpp>

#include <termios.h>
#include <unistd.h>

#include <cstdio>

namespace cam_loc_ros {

namespace {

termios g_orig_termios{};
bool g_termios_saved = false;

void restoreTerminal() {
  if (g_termios_saved) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
    g_termios_saved = false;
  }
}

}  // namespace

KeyboardInput::KeyboardInput() = default;

KeyboardInput::~KeyboardInput() { stop(); }

void KeyboardInput::start() {
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

  thread_ = std::thread([this]() { run(); });
}

void KeyboardInput::stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) {
    thread_.join();
  }
  restoreTerminal();
}

KeyboardInput::Command KeyboardInput::poll() {
  const int cmd = pending_.exchange(static_cast<int>(Command::kNone));
  return static_cast<Command>(cmd);
}

void KeyboardInput::run() {
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
