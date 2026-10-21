// Typed, length-prefixed policy wire protocol and recorded observation files.
#ifndef ROVI_POLICY_WIRE_HPP_
#define ROVI_POLICY_WIRE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rovi::policy {

inline constexpr std::uint32_t kMagic = 0x50564F52;
inline constexpr std::uint16_t kVersion = 1;
inline constexpr std::uint32_t kMaximumPayload = 16U * 1024U * 1024U;
inline constexpr std::size_t kHeaderBytes = 12;

inline constexpr std::size_t kSha256Bytes = 32;

enum class FrameType : std::uint8_t { kHello = 1, kRequest = 2, kResponse = 3, kError = 4 };

using Bytes = std::vector<std::uint8_t>;
using Sha256 = std::array<std::uint8_t, kSha256Bytes>;
using PolicyId = Sha256;

struct Header {
  FrameType type{FrameType::kHello};
  std::uint32_t length{0};

  bool operator==(const Header&) const = default;
};

struct Observation {
  std::vector<float> state;
  std::uint32_t width{0};
  std::uint32_t height{0};
  Bytes rgb;

  bool operator==(const Observation&) const = default;
};

struct Hello {
  PolicyId policy{};
  Sha256 adapter{};
  std::uint32_t action_steps{0};
  std::uint32_t action_dim{0};
  std::uint32_t state_dim{0};
  std::uint32_t image_width{0};
  std::uint32_t image_height{0};
  std::int64_t action_period_ns{0};
  std::string policy_type;
  std::string task;
  std::string robot_type;
  std::string device;
  std::string runtime;

  bool operator==(const Hello&) const = default;
};

struct Request {
  std::uint64_t id{0};
  std::int64_t observation_ns{0};
  Observation observation;

  bool operator==(const Request&) const = default;
};

struct Response {
  std::uint64_t id{0};
  std::int64_t observation_ns{0};
  PolicyId policy{};
  std::uint64_t noise_seed{0};
  std::uint64_t inference_ns{0};
  std::uint64_t handling_ns{0};
  std::uint32_t steps{0};
  std::uint32_t dim{0};
  std::vector<float> actions;

  bool operator==(const Response&) const = default;
};

struct ErrorReply {
  std::uint64_t id{0};
  std::uint16_t code{0};
  std::string message;

  bool operator==(const ErrorReply&) const = default;
};

struct RecordedObservation {
  std::uint32_t episode{0};
  std::uint32_t frame{0};
  Observation observation;
};

[[nodiscard]] Bytes encode_header(FrameType type, std::uint32_t length);
[[nodiscard]] std::optional<Header> decode_header(std::span<const std::uint8_t> bytes);

[[nodiscard]] std::optional<Bytes> encode_hello(const Hello& hello);
[[nodiscard]] std::optional<Bytes> encode_request(const Request& request);
[[nodiscard]] std::optional<Bytes> encode_response(const Response& response);
[[nodiscard]] std::optional<Bytes> encode_error(const ErrorReply& error);
[[nodiscard]] std::optional<Hello> decode_hello(std::span<const std::uint8_t> payload);
[[nodiscard]] std::optional<Request> decode_request(std::span<const std::uint8_t> payload);
[[nodiscard]] std::optional<Response> decode_response(std::span<const std::uint8_t> payload);
[[nodiscard]] std::optional<ErrorReply> decode_error(std::span<const std::uint8_t> payload);

// Payload sizes, which are also the modeled upload and download bytes of a job.
[[nodiscard]] std::size_t request_bytes(const Observation& observation);
[[nodiscard]] std::size_t response_bytes(std::uint32_t steps, std::uint32_t dim);

[[nodiscard]] std::optional<RecordedObservation> read_observation_file(
    const std::filesystem::path& path);
[[nodiscard]] bool write_observation_file(const std::filesystem::path& path,
                                          const RecordedObservation& recorded);

[[nodiscard]] std::string to_hex(const Sha256& digest);
[[nodiscard]] std::optional<PolicyId> policy_from_hex(std::string_view hex);

}  // namespace rovi::policy

#endif  // ROVI_POLICY_WIRE_HPP_
