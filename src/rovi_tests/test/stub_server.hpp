// The adapter's stub policy server, run as a child process for 4 x 2 images on a port the system
// chooses.
#ifndef ROVI_TESTS_STUB_SERVER_HPP_
#define ROVI_TESTS_STUB_SERVER_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <signal.h>  // NOLINT(modernize-deprecated-headers): kill() is POSIX only.
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace rovi::tests {

inline constexpr std::string_view kStubPolicy =
    "8fbdecccd995c05c66e526de59937b283d13db45016376739783b54be2b5b912";

class StubServer {
 public:
  StubServer(const std::string& name, int delay_ms, const std::vector<std::string>& extra = {}) {
    const std::filesystem::path port_file =
        std::filesystem::temp_directory_path() /
        ("rovi_stub_port_" + std::to_string(::getpid()) + "_" + name);
    std::filesystem::remove(port_file);
    std::vector<std::string> arguments{"python3",
                                       std::string{ROVI_POLICY_ADAPTER} + "/rovi_policy_server.py",
                                       "--stub",
                                       "--host",
                                       "127.0.0.1",
                                       "--port",
                                       "0",
                                       "--port-file",
                                       port_file.string(),
                                       "--task",
                                       "test",
                                       "--image-width",
                                       "4",
                                       "--image-height",
                                       "2",
                                       "--stub-delay-ms",
                                       std::to_string(delay_ms)};
    arguments.insert(arguments.end(), extra.begin(), extra.end());
    std::vector<char*> argv(arguments.size() + 1, nullptr);
    std::ranges::transform(arguments, argv.begin(),
                           [](std::string& argument) { return std::span<char>{argument}.data(); });
    if (::posix_spawnp(&pid_, "python3", nullptr, nullptr, argv.data(), environ) != 0) {
      pid_ = -1;
      return;
    }
    const auto deadline = std::chrono::steady_clock::now() + kStartLimit;
    while (port_ == 0 && std::chrono::steady_clock::now() < deadline) {
      std::ifstream stream{port_file};
      int bound = 0;
      if (stream >> bound) {
        port_ = static_cast<std::uint16_t>(bound);
      } else {
        std::this_thread::sleep_for(kPortPoll);
      }
    }
  }
  StubServer(const StubServer&) = delete;
  StubServer& operator=(const StubServer&) = delete;
  StubServer(StubServer&&) = delete;
  StubServer& operator=(StubServer&&) = delete;
  ~StubServer() { stop(); }

  [[nodiscard]] std::uint16_t port() const { return port_; }

  void stop() {
    if (pid_ > 0) {
      static_cast<void>(::kill(pid_, SIGTERM));
      static_cast<void>(::waitpid(pid_, nullptr, 0));
      pid_ = -1;
    }
  }

 private:
  static constexpr std::chrono::seconds kStartLimit{20};
  static constexpr std::chrono::milliseconds kPortPoll{20};

  pid_t pid_{-1};
  std::uint16_t port_{0};
};

}  // namespace rovi::tests

#endif  // ROVI_TESTS_STUB_SERVER_HPP_
