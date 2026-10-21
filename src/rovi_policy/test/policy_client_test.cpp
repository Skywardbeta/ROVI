#include "rovi_policy/policy_client.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "rovi_policy/wire.hpp"

namespace rovi::policy {
namespace {

constexpr ClientTiming kTiming{.connect = std::chrono::milliseconds{1'000},
                               .reply = std::chrono::milliseconds{1'000}};

PolicyId id_of(std::uint8_t fill) {
  PolicyId id{};
  id.fill(fill);
  return id;
}

std::pair<Socket, Socket> connected_pair() {
  std::array<int, 2> descriptors{-1, -1};
  EXPECT_EQ(::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, descriptors.data()), 0);
  return {Socket{descriptors.at(0)}, Socket{descriptors.at(1)}};
}

void send_frame(const Socket& socket, FrameType type, const Bytes& payload) {
  Bytes frame = encode_header(type, static_cast<std::uint32_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());
  ASSERT_EQ(::send(socket.get(), frame.data(), frame.size(), MSG_NOSIGNAL),
            static_cast<ssize_t>(frame.size()));
}

std::optional<Request> receive_request(const Socket& socket) {
  Bytes header(kHeaderBytes);
  if (::recv(socket.get(), header.data(), header.size(), MSG_WAITALL) !=
      static_cast<ssize_t>(header.size())) {
    return std::nullopt;
  }
  const std::optional<Header> decoded = decode_header(header);
  if (!decoded || decoded->type != FrameType::kRequest) {
    return std::nullopt;
  }
  Bytes payload(decoded->length);
  if (::recv(socket.get(), payload.data(), payload.size(), MSG_WAITALL) !=
      static_cast<ssize_t>(payload.size())) {
    return std::nullopt;
  }
  return decode_request(payload);
}

Hello hello_for(const PolicyId& policy) {
  return Hello{.policy = policy,
               .action_steps = 2,
               .action_dim = 3,
               .state_dim = 3,
               .image_width = 2,
               .image_height = 1,
               .action_period_ns = 33'333'333,
               .policy_type = "stub",
               .task = "pick",
               .robot_type = "so_follower",
               .device = "cpu",
               .runtime = "stub"};
}

Request request_for(std::uint64_t id) {
  return Request{
      .id = id,
      .observation_ns = 500,
      .observation = {
          .state = {1.0F, 2.0F, 3.0F}, .width = 2, .height = 1, .rgb = {1, 2, 3, 4, 5, 6}}};
}

Response response_for(const Request& request, const PolicyId& policy) {
  return Response{.id = request.id,
                  .observation_ns = request.observation_ns,
                  .policy = policy,
                  .noise_seed = request.id,
                  .inference_ns = 900,
                  .handling_ns = 950,
                  .steps = 2,
                  .dim = 3,
                  .actions = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F}};
}

TEST(PolicyClient, ReturnsTheValidatedResponseToItsRequest) {
  auto [client_end, server_end] = connected_pair();
  const std::jthread server{[&server_end] {
    send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(1))));
    const std::optional<Request> request = receive_request(server_end);
    ASSERT_TRUE(request.has_value());
    send_frame(server_end, FrameType::kResponse,
               *encode_response(response_for(*request, id_of(1))));
  }};
  PolicyClient client =
      PolicyClient::over(std::move(client_end), {.policy = id_of(1), .task = "pick"}, kTiming);
  const CallResult result = client.infer(request_for(9), std::stop_source{}.get_token());
  ASSERT_EQ(result.status, CallStatus::kOk) << result.detail;
  EXPECT_EQ(client.connections(), 1U);
  EXPECT_EQ(result.response, response_for(request_for(9), id_of(1)));
  EXPECT_EQ(client.hello(), hello_for(id_of(1)));
}

TEST(PolicyClient, RefusesAServerServingAnotherPolicy) {
  auto [client_end, server_end] = connected_pair();
  send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(2))));
  PolicyClient client = PolicyClient::over(std::move(client_end),
                                           {.policy = id_of(1), .task = std::nullopt}, kTiming);
  EXPECT_EQ(client.infer(request_for(1), std::stop_source{}.get_token()).status,
            CallStatus::kIdentity);
  EXPECT_FALSE(client.hello().has_value());
}

TEST(PolicyClient, RefusesAServerWithAnotherTaskAndCountsOnlyAcceptedHandshakes) {
  auto [client_end, server_end] = connected_pair();
  send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(1))));
  PolicyClient client =
      PolicyClient::over(std::move(client_end), {.policy = id_of(1), .task = "place"}, kTiming);
  const CallResult result = client.infer(request_for(1), std::stop_source{}.get_token());
  EXPECT_EQ(result.status, CallStatus::kIdentity);
  EXPECT_EQ(result.detail, "the server's task is \"pick\"");
  EXPECT_EQ(client.connections(), 0U);
}

TEST(PolicyClient, RejectsAMalformedHello) {
  auto [client_end, server_end] = connected_pair();
  send_frame(server_end, FrameType::kHello, Bytes(5, 0));
  PolicyClient client = PolicyClient::over(std::move(client_end),
                                           {.policy = id_of(1), .task = std::nullopt}, kTiming);
  EXPECT_EQ(client.infer(request_for(1), std::stop_source{}.get_token()).status,
            CallStatus::kProtocol);
}

TEST(PolicyClient, RefusesAReplyThatDoesNotEchoTheRequestAndDropsTheConnection) {
  auto [client_end, server_end] = connected_pair();
  const std::jthread server{[&server_end] {
    send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(1))));
    const std::optional<Request> request = receive_request(server_end);
    ASSERT_TRUE(request.has_value());
    Response wrong = response_for(*request, id_of(1));
    wrong.id += 1;
    send_frame(server_end, FrameType::kResponse, *encode_response(wrong));
  }};
  PolicyClient client = PolicyClient::over(std::move(client_end),
                                           {.policy = id_of(1), .task = std::nullopt}, kTiming);
  EXPECT_EQ(client.infer(request_for(4), std::stop_source{}.get_token()).status,
            CallStatus::kProtocol);
  EXPECT_EQ(client.infer(request_for(5), std::stop_source{}.get_token()).status,
            CallStatus::kUnreachable);
}

TEST(PolicyClient, RefusesAReplyFromAnotherPolicyOrOfAnotherShape) {
  for (const bool other_policy : {true, false}) {
    auto [client_end, server_end] = connected_pair();
    const std::jthread server{[&server_end, other_policy] {
      send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(1))));
      const std::optional<Request> request = receive_request(server_end);
      ASSERT_TRUE(request.has_value());
      Response reply = response_for(*request, id_of(other_policy ? 3 : 1));
      if (!other_policy) {
        reply.steps = 3;
        reply.dim = 2;
      }
      send_frame(server_end, FrameType::kResponse, *encode_response(reply));
    }};
    PolicyClient client = PolicyClient::over(std::move(client_end),
                                             {.policy = id_of(1), .task = std::nullopt}, kTiming);
    EXPECT_EQ(client.infer(request_for(4), std::stop_source{}.get_token()).status,
              other_policy ? CallStatus::kIdentity : CallStatus::kProtocol);
  }
}

TEST(PolicyClient, KeepsTheConnectionAfterARemoteError) {
  auto [client_end, server_end] = connected_pair();
  const std::jthread server{[&server_end] {
    send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(1))));
    const std::optional<Request> first = receive_request(server_end);
    ASSERT_TRUE(first.has_value());
    send_frame(server_end, FrameType::kError,
               *encode_error({.id = first->id, .code = 3, .message = "busy"}));
    const std::optional<Request> second = receive_request(server_end);
    ASSERT_TRUE(second.has_value());
    send_frame(server_end, FrameType::kResponse, *encode_response(response_for(*second, id_of(1))));
  }};
  PolicyClient client = PolicyClient::over(std::move(client_end),
                                           {.policy = id_of(1), .task = std::nullopt}, kTiming);
  const CallResult failed = client.infer(request_for(1), std::stop_source{}.get_token());
  EXPECT_EQ(failed.status, CallStatus::kRemoteError);
  EXPECT_EQ(failed.detail, "busy");
  EXPECT_EQ(client.infer(request_for(2), std::stop_source{}.get_token()).status, CallStatus::kOk);
}

TEST(PolicyClient, TimesOutWhenTheServerNeverReplies) {
  auto [client_end, server_end] = connected_pair();
  send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(1))));
  PolicyClient client =
      PolicyClient::over(std::move(client_end), {.policy = id_of(1), .task = std::nullopt},
                         ClientTiming{.connect = std::chrono::milliseconds{1'000},
                                      .reply = std::chrono::milliseconds{100}});
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(client.infer(request_for(1), std::stop_source{}.get_token()).status,
            CallStatus::kTimeout);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds{900});
}

TEST(PolicyClient, StopsWaitingWhenAskedTo) {
  auto [client_end, server_end] = connected_pair();
  send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(1))));
  PolicyClient client = PolicyClient::over(std::move(client_end),
                                           {.policy = id_of(1), .task = std::nullopt}, kTiming);
  std::stop_source stop;
  const std::jthread stopper{[&stop] {
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
    stop.request_stop();
  }};
  EXPECT_EQ(client.infer(request_for(1), stop.get_token()).status, CallStatus::kStopped);
}

TEST(PolicyClient, RefusesAnObservationThePolicyCannotTake) {
  auto [client_end, server_end] = connected_pair();
  send_frame(server_end, FrameType::kHello, *encode_hello(hello_for(id_of(1))));
  PolicyClient client = PolicyClient::over(std::move(client_end),
                                           {.policy = id_of(1), .task = std::nullopt}, kTiming);
  Request request = request_for(1);
  request.observation.state.pop_back();
  EXPECT_EQ(client.infer(request, std::stop_source{}.get_token()).status, CallStatus::kProtocol);
}

TEST(PolicyClient, ReportsAnUnreachableServer) {
  PolicyClient client{Endpoint{.host = "127.0.0.1", .port = 1},
                      {.policy = id_of(1), .task = std::nullopt},
                      kTiming};
  EXPECT_EQ(client.infer(request_for(1), std::stop_source{}.get_token()).status,
            CallStatus::kUnreachable);
}

}  // namespace
}  // namespace rovi::policy
