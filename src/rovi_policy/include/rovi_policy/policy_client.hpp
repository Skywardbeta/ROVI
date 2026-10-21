// Blocking TCP client of one policy server: one request at a time, identity- and echo-checked.
#ifndef ROVI_POLICY_POLICY_CLIENT_HPP_
#define ROVI_POLICY_POLICY_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>

#include "rovi_policy/wire.hpp"

namespace rovi::policy {

struct Endpoint {
  std::string host;
  std::uint16_t port{0};
};

// The server a client accepts: its checkpoint, and its task when one is given.
struct ExpectedServer {
  PolicyId policy{};
  std::optional<std::string> task;
};

struct ClientTiming {
  std::chrono::milliseconds connect{0};
  std::chrono::milliseconds reply{0};
};

enum class CallStatus : std::uint8_t {
  kOk,
  kUnreachable,
  kTimeout,
  kProtocol,
  kIdentity,
  kRemoteError,
  kStopped
};

struct CallResult {
  CallStatus status{CallStatus::kProtocol};
  std::optional<Response> response;
  std::string detail;
};

[[nodiscard]] std::string_view status_name(CallStatus status);

class Socket {
 public:
  Socket() = default;
  explicit Socket(int descriptor);
  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;
  Socket(Socket&& other) noexcept;
  Socket& operator=(Socket&& other) noexcept;
  ~Socket();

  [[nodiscard]] int get() const;
  [[nodiscard]] bool valid() const;
  void reset();

 private:
  int descriptor_{-1};
};

class PolicyClient {
 public:
  PolicyClient(Endpoint endpoint, ExpectedServer expected, ClientTiming timing);
  // A client bound to one connected socket that never reconnects.
  [[nodiscard]] static PolicyClient over(Socket connected, ExpectedServer expected,
                                         ClientTiming timing);

  // Connects when needed and returns one validated response; any failure drops the connection.
  [[nodiscard]] CallResult infer(const Request& request, const std::stop_token& stop);
  [[nodiscard]] const std::optional<Hello>& hello() const;
  // Accepted handshakes so far, and the latest accepted server, kept after its connection drops.
  [[nodiscard]] std::uint32_t connections() const;
  [[nodiscard]] const std::optional<Hello>& accepted() const;

 private:
  [[nodiscard]] CallResult open(const std::stop_token& stop);
  [[nodiscard]] CallResult greet(const std::stop_token& stop);
  [[nodiscard]] CallResult exchange(const Request& request, const std::stop_token& stop);

  Endpoint endpoint_;
  ExpectedServer expected_;
  ClientTiming timing_;
  Socket socket_;
  std::optional<Hello> hello_;
  std::uint32_t connections_{0};
  std::optional<Hello> accepted_;
};

}  // namespace rovi::policy

#endif  // ROVI_POLICY_POLICY_CLIENT_HPP_
