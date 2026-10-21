#include "rovi_policy/wire.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

namespace rovi::policy {
namespace {

// Encoded by adapters/lerobot/rovi_wire.py, so these pin the two implementations together.
constexpr std::string_view kPythonHello =
    "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e"
    "2f"
    "303132333435363738393a3b3c3d3e3f020000000300000003000000020000000100000055a0fc0100000000070073"
    "6d"
    "6f6c766c6104007069636b0b00736f5f666f6c6c6f7765720400637564610d006c65726f626f7420302e342e34";
constexpr std::string_view kPythonResponse =
    "0700000000000000fbffffffffffffff000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e"
    "1f"
    "07000000000000008403000000000000b60300000000000002000000030000000000803f000020c000005040000000"
    "00"
    "0000c842000000be";
constexpr std::string_view kPythonObservationFile =
    "524f56494f4253311b00000090010000030000000000c03f000000c00000404002000000010000000102030405"
    "06";

Bytes from_hex(std::string_view hex) {
  Bytes bytes;
  for (std::size_t index = 0; index + 1 < hex.size(); index += 2) {
    bytes.push_back(
        static_cast<std::uint8_t>(std::stoi(std::string{hex.substr(index, 2)}, nullptr, 16)));
  }
  return bytes;
}

Sha256 counting_id(std::size_t first = 0) {
  Sha256 id{};
  for (std::size_t index = 0; index < id.size(); ++index) {
    id.at(index) = static_cast<std::uint8_t>(first + index);
  }
  return id;
}

Observation tiny_observation() {
  return Observation{
      .state = {1.5F, -2.0F, 3.0F}, .width = 2, .height = 1, .rgb = {1, 2, 3, 4, 5, 6}};
}

std::filesystem::path write_file(const std::string& name, const Bytes& bytes) {
  const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
  std::ofstream stream{path, std::ios::binary};
  for (const std::uint8_t byte : bytes) {
    stream.put(static_cast<char>(byte));
  }
  return path;
}

TEST(PolicyWire, DecodesAndReencodesAHelloFromThePythonAdapter) {
  const Bytes encoded = from_hex(kPythonHello);
  const std::optional<Hello> hello = decode_hello(encoded);
  ASSERT_TRUE(hello.has_value());
  EXPECT_EQ(*hello, (Hello{.policy = counting_id(),
                           .adapter = counting_id(32),
                           .action_steps = 2,
                           .action_dim = 3,
                           .state_dim = 3,
                           .image_width = 2,
                           .image_height = 1,
                           .action_period_ns = 33'333'333,
                           .policy_type = "smolvla",
                           .task = "pick",
                           .robot_type = "so_follower",
                           .device = "cuda",
                           .runtime = "lerobot 0.4.4"}));
  EXPECT_EQ(encode_hello(*hello), encoded);
}

TEST(PolicyWire, DecodesAndReencodesAResponseFromThePythonAdapter) {
  const Bytes encoded = from_hex(kPythonResponse);
  const std::optional<Response> response = decode_response(encoded);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(*response, (Response{.id = 7,
                                 .observation_ns = -5,
                                 .policy = counting_id(),
                                 .noise_seed = 7,
                                 .inference_ns = 900,
                                 .handling_ns = 950,
                                 .steps = 2,
                                 .dim = 3,
                                 .actions = {1.0F, -2.5F, 3.25F, 0.0F, 100.0F, -0.125F}}));
  EXPECT_EQ(encode_response(*response), encoded);
  EXPECT_EQ(response_bytes(2, 3), encoded.size());
}

TEST(PolicyWire, ReadsAnObservationFileFromThePythonAdapter) {
  const Bytes contents = from_hex(kPythonObservationFile);
  const std::optional<RecordedObservation> recorded =
      read_observation_file(write_file("rovi_policy_wire_test.rovobs", contents));
  ASSERT_TRUE(recorded.has_value());
  EXPECT_EQ(recorded->episode, 27U);
  EXPECT_EQ(recorded->frame, 400U);
  EXPECT_EQ(recorded->observation, tiny_observation());
  EXPECT_EQ(request_bytes(recorded->observation), contents.size());
}

TEST(PolicyWire, WritesTheSameObservationFileAsThePythonAdapter) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "rovi_policy_wire_written.rovobs";
  ASSERT_TRUE(write_observation_file(
      path, RecordedObservation{.episode = 27, .frame = 400, .observation = tiny_observation()}));
  const std::optional<RecordedObservation> recorded = read_observation_file(path);
  ASSERT_TRUE(recorded.has_value());
  EXPECT_EQ(recorded->observation, tiny_observation());
  EXPECT_EQ(std::filesystem::file_size(path), from_hex(kPythonObservationFile).size());
}

TEST(PolicyWire, RefusesAFileWithoutTheObservationMagic) {
  Bytes contents = from_hex(kPythonObservationFile);
  contents.front() = 'X';
  EXPECT_FALSE(read_observation_file(write_file("rovi_policy_wire_bad.rovobs", contents)));
  EXPECT_FALSE(read_observation_file("/nonexistent/rovi.rovobs"));
}

TEST(PolicyWire, RequestsRoundTripAtTheirDeclaredSize) {
  const Request request{.id = (std::uint64_t{3} << 32U) | 9U,
                        .observation_ns = 1'234,
                        .observation = tiny_observation()};
  const std::optional<Bytes> encoded = encode_request(request);
  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(encoded->size(), request_bytes(request.observation));
  EXPECT_EQ(decode_request(*encoded), request);
}

TEST(PolicyWire, RejectsTruncatedOrPaddedPayloads) {
  Bytes response = from_hex(kPythonResponse);
  response.push_back(0);
  EXPECT_FALSE(decode_response(response));
  response.resize(response.size() - 2);
  EXPECT_FALSE(decode_response(response));
  Bytes hello = from_hex(kPythonHello);
  hello.pop_back();
  EXPECT_FALSE(decode_hello(hello));
  EXPECT_FALSE(decode_request(Bytes(10, 0)));
}

TEST(PolicyWire, RejectsHeadersWithAnotherMagicVersionTypeOrOversizedLength) {
  const Bytes header = encode_header(FrameType::kResponse, 80);
  EXPECT_EQ(decode_header(header), (Header{.type = FrameType::kResponse, .length = 80}));
  for (const std::size_t corrupted : {std::size_t{0}, std::size_t{4}, std::size_t{6}}) {
    Bytes bad = header;
    bad.at(corrupted) = 0x7F;
    EXPECT_FALSE(decode_header(bad)) << corrupted;
  }
  EXPECT_FALSE(decode_header(encode_header(FrameType::kHello, kMaximumPayload + 1)));
  EXPECT_FALSE(decode_header(Bytes(kHeaderBytes - 1, 0)));
}

TEST(PolicyWire, RefusesToEncodeAnImageOrChunkThatDoesNotMatchItsSize) {
  Observation observation = tiny_observation();
  observation.rgb.pop_back();
  EXPECT_FALSE(encode_request(Request{.id = 1, .observation_ns = 0, .observation = observation}));
  EXPECT_FALSE(encode_response(Response{.steps = 2, .dim = 3, .actions = {1.0F}}));
}

TEST(PolicyWire, ErrorsRoundTrip) {
  const ErrorReply error{.id = 5, .code = 2, .message = "expected state 6"};
  const std::optional<Bytes> encoded = encode_error(error);
  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(decode_error(*encoded), error);
}

TEST(PolicyWire, PolicyIdsRoundTripThroughLowercaseHexOnly) {
  const std::string hex = to_hex(counting_id());
  EXPECT_EQ(hex, "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
  EXPECT_EQ(policy_from_hex(hex), counting_id());
  EXPECT_FALSE(policy_from_hex(hex.substr(1)));
  EXPECT_FALSE(policy_from_hex("0A" + hex.substr(2)));
  EXPECT_FALSE(policy_from_hex("zz" + hex.substr(2)));
}

}  // namespace
}  // namespace rovi::policy
