#include "rovi_policy/policy_client.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <utility>

#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "rovi_policy/wire.hpp"

namespace rovi::policy {
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::chrono::milliseconds kPollSlice{50};

enum class Wait : std::uint8_t { kReady, kTimeout, kStopped, kFailed };

Wait wait_for(int descriptor, short events, Clock::time_point deadline,
              const std::stop_token& stop) {
  while (true) {
    if (stop.stop_requested()) {
      return Wait::kStopped;
    }
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (remaining.count() <= 0) {
      return Wait::kTimeout;
    }
    pollfd entry{.fd = descriptor, .events = events, .revents = 0};
    const int ready = ::poll(&entry, 1, static_cast<int>(std::min(remaining, kPollSlice).count()));
    if (ready < 0 && errno != EINTR) {
      return Wait::kFailed;
    }
    if (ready > 0) {
      return (entry.revents & events) != 0 ? Wait::kReady : Wait::kFailed;
    }
  }
}

CallResult failure(CallStatus status, std::string detail) {
  return CallResult{.status = status, .response = std::nullopt, .detail = std::move(detail)};
}

CallResult wait_failure(Wait wait, std::string_view what) {
  switch (wait) {
    case Wait::kTimeout:
      return failure(CallStatus::kTimeout, std::string{what} + " timed out");
    case Wait::kStopped:
      return failure(CallStatus::kStopped, std::string{what} + " stopped");
    case Wait::kReady:
    case Wait::kFailed:
      break;
  }
  return failure(CallStatus::kUnreachable, std::string{what} + " failed");
}

std::optional<CallResult> send_all(int descriptor, std::span<const std::uint8_t> bytes,
                                   Clock::time_point deadline, const std::stop_token& stop) {
  std::size_t sent = 0;
  while (sent < bytes.size()) {
    const Wait wait = wait_for(descriptor, POLLOUT, deadline, stop);
    if (wait != Wait::kReady) {
      return wait_failure(wait, "sending");
    }
    const std::span<const std::uint8_t> rest = bytes.subspan(sent);
    const ssize_t count = ::send(descriptor, rest.data(), rest.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      return failure(CallStatus::kUnreachable, "the connection dropped while sending");
    }
    sent += count > 0 ? static_cast<std::size_t>(count) : 0U;
  }
  return std::nullopt;
}

std::optional<CallResult> receive_exact(int descriptor, std::span<std::uint8_t> bytes,
                                        Clock::time_point deadline, const std::stop_token& stop) {
  std::size_t received = 0;
  while (received < bytes.size()) {
    const Wait wait = wait_for(descriptor, POLLIN, deadline, stop);
    if (wait != Wait::kReady) {
      return wait_failure(wait, "receiving");
    }
    const std::span<std::uint8_t> rest = bytes.subspan(received);
    const ssize_t count = ::recv(descriptor, rest.data(), rest.size(), MSG_DONTWAIT);
    if (count == 0) {
      return failure(CallStatus::kUnreachable, "the server closed the connection");
    }
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      return failure(CallStatus::kUnreachable, "the connection dropped while receiving");
    }
    received += count > 0 ? static_cast<std::size_t>(count) : 0U;
  }
  return std::nullopt;
}

struct Frame {
  Header header;
  Bytes payload;
};

std::optional<CallResult> receive_frame(int descriptor, Frame& frame, Clock::time_point deadline,
                                        const std::stop_token& stop) {
  Bytes header(kHeaderBytes);
  if (std::optional<CallResult> failed = receive_exact(descriptor, header, deadline, stop)) {
    return failed;
  }
  const std::optional<Header> decoded = decode_header(header);
  if (!decoded) {
    return failure(CallStatus::kProtocol, "the server sent a malformed frame header");
  }
  frame.header = *decoded;
  frame.payload.assign(decoded->length, 0);
  return receive_exact(descriptor, frame.payload, deadline, stop);
}

Socket connect_to(const Endpoint& endpoint, Clock::time_point deadline,
                  const std::stop_token& stop) {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* found = nullptr;
  if (::getaddrinfo(endpoint.host.c_str(), std::to_string(endpoint.port).c_str(), &hints, &found) !=
      0) {
    return Socket{};
  }
  const std::unique_ptr<addrinfo, decltype(&::freeaddrinfo)> list{found, &::freeaddrinfo};
  for (const addrinfo* entry = list.get(); entry != nullptr; entry = entry->ai_next) {
    Socket socket{::socket(entry->ai_family, entry->ai_socktype | SOCK_NONBLOCK | SOCK_CLOEXEC,
                           entry->ai_protocol)};
    if (!socket.valid()) {
      continue;
    }
    if (::connect(socket.get(), entry->ai_addr, entry->ai_addrlen) != 0 && errno != EINPROGRESS) {
      continue;
    }
    if (wait_for(socket.get(), POLLOUT, deadline, stop) != Wait::kReady) {
      continue;
    }
    // A second connect reports how the first ended: EISCONN once connected.
    if (::connect(socket.get(), entry->ai_addr, entry->ai_addrlen) != 0 && errno != EISCONN) {
      continue;
    }
    const int enabled = 1;
    static_cast<void>(
        ::setsockopt(socket.get(), IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)));
    return socket;
  }
  return Socket{};
}

}  // namespace

std::string_view status_name(CallStatus status) {
  switch (status) {
    case CallStatus::kOk:
      return "ok";
    case CallStatus::kUnreachable:
      return "unreachable";
    case CallStatus::kTimeout:
      return "timeout";
    case CallStatus::kProtocol:
      return "protocol";
    case CallStatus::kIdentity:
      return "identity";
    case CallStatus::kRemoteError:
      return "remote error";
    case CallStatus::kStopped:
      return "stopped";
  }
  return "unknown";
}

Socket::Socket(int descriptor) : descriptor_{descriptor} {}

Socket::Socket(Socket&& other) noexcept : descriptor_{std::exchange(other.descriptor_, -1)} {}

Socket& Socket::operator=(Socket&& other) noexcept {
  if (this != &other) {
    reset();
    descriptor_ = std::exchange(other.descriptor_, -1);
  }
  return *this;
}

Socket::~Socket() { reset(); }

int Socket::get() const { return descriptor_; }

bool Socket::valid() const { return descriptor_ >= 0; }

void Socket::reset() {
  if (descriptor_ >= 0) {
    static_cast<void>(::close(descriptor_));
    descriptor_ = -1;
  }
}

PolicyClient::PolicyClient(Endpoint endpoint, ExpectedServer expected, ClientTiming timing)
    : endpoint_{std::move(endpoint)}, expected_{std::move(expected)}, timing_{timing} {}

PolicyClient PolicyClient::over(Socket connected, ExpectedServer expected, ClientTiming timing) {
  PolicyClient client{Endpoint{}, std::move(expected), timing};
  client.socket_ = std::move(connected);
  return client;
}

const std::optional<Hello>& PolicyClient::hello() const { return hello_; }

std::uint32_t PolicyClient::connections() const { return connections_; }

const std::optional<Hello>& PolicyClient::accepted() const { return accepted_; }

CallResult PolicyClient::infer(const Request& request, const std::stop_token& stop) {
  if (!hello_) {
    CallResult opened = open(stop);
    if (opened.status != CallStatus::kOk) {
      socket_.reset();
      hello_.reset();
      return opened;
    }
  }
  CallResult result = exchange(request, stop);
  if (result.status != CallStatus::kOk && result.status != CallStatus::kRemoteError) {
    socket_.reset();
    hello_.reset();
  }
  return result;
}

CallResult PolicyClient::open(const std::stop_token& stop) {
  if (!socket_.valid()) {
    if (endpoint_.host.empty()) {
      return failure(CallStatus::kUnreachable, "the connection is closed");
    }
    socket_ = connect_to(endpoint_, Clock::now() + timing_.connect, stop);
    if (!socket_.valid()) {
      return failure(stop.stop_requested() ? CallStatus::kStopped : CallStatus::kUnreachable,
                     "cannot connect to " + endpoint_.host + ":" + std::to_string(endpoint_.port));
    }
  }
  return greet(stop);
}

CallResult PolicyClient::greet(const std::stop_token& stop) {
  Frame frame;
  if (std::optional<CallResult> failed =
          receive_frame(socket_.get(), frame, Clock::now() + timing_.connect, stop)) {
    return *failed;
  }
  std::optional<Hello> greeting =
      frame.header.type == FrameType::kHello ? decode_hello(frame.payload) : std::nullopt;
  if (!greeting) {
    return failure(CallStatus::kProtocol, "the server did not greet with a valid hello");
  }
  if (greeting->policy != expected_.policy) {
    return failure(CallStatus::kIdentity, "the server serves policy " + to_hex(greeting->policy) +
                                              ", not " + to_hex(expected_.policy));
  }
  if (expected_.task && greeting->task != *expected_.task) {
    return failure(CallStatus::kIdentity, "the server's task is \"" + greeting->task + "\"");
  }
  hello_ = std::move(*greeting);
  accepted_ = hello_;
  ++connections_;
  return CallResult{.status = CallStatus::kOk, .response = std::nullopt, .detail = {}};
}

CallResult PolicyClient::exchange(const Request& request, const std::stop_token& stop) {
  if (!hello_) {
    return failure(CallStatus::kProtocol, "no hello was received");
  }
  const Observation& observation = request.observation;
  if (observation.state.size() != hello_->state_dim || observation.width != hello_->image_width ||
      observation.height != hello_->image_height) {
    return failure(CallStatus::kProtocol, "the observation does not match the policy's inputs");
  }
  const std::optional<Bytes> payload = encode_request(request);
  if (!payload) {
    return failure(CallStatus::kProtocol, "the request cannot be encoded");
  }
  const Clock::time_point deadline = Clock::now() + timing_.reply;
  const Bytes header =
      encode_header(FrameType::kRequest, static_cast<std::uint32_t>(payload->size()));
  if (std::optional<CallResult> failed = send_all(socket_.get(), header, deadline, stop)) {
    return *failed;
  }
  if (std::optional<CallResult> failed = send_all(socket_.get(), *payload, deadline, stop)) {
    return *failed;
  }
  Frame frame;
  if (std::optional<CallResult> failed = receive_frame(socket_.get(), frame, deadline, stop)) {
    return *failed;
  }
  if (frame.header.type == FrameType::kError) {
    const std::optional<ErrorReply> error = decode_error(frame.payload);
    if (!error || error->id != request.id) {
      return failure(CallStatus::kProtocol, "the server sent an unmatched error");
    }
    return failure(CallStatus::kRemoteError, error->message);
  }
  std::optional<Response> response =
      frame.header.type == FrameType::kResponse ? decode_response(frame.payload) : std::nullopt;
  if (!response) {
    return failure(CallStatus::kProtocol, "the server sent no valid reply");
  }
  if (response->id != request.id || response->observation_ns != request.observation_ns) {
    return failure(CallStatus::kProtocol, "the server's reply does not echo the request");
  }
  if (response->policy != expected_.policy) {
    return failure(CallStatus::kIdentity, "the reply came from policy " + to_hex(response->policy));
  }
  if (response->steps != hello_->action_steps || response->dim != hello_->action_dim) {
    return failure(CallStatus::kProtocol, "the reply's chunk shape differs from the hello");
  }
  return CallResult{.status = CallStatus::kOk, .response = std::move(response), .detail = {}};
}

}  // namespace rovi::policy
