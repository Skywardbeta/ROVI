#include "rovi_policy/wire.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace rovi::policy {
namespace {

static_assert(std::numeric_limits<float>::is_iec559);

constexpr std::size_t kByteBits = 8;
constexpr std::size_t kChannels = 3;
constexpr std::size_t kRequestFixedBytes = 8 + 8 + 4 + 4 + 4;
constexpr std::size_t kResponseFixedBytes = 8 + 8 + kSha256Bytes + 8 + 8 + 8 + 4 + 4;
constexpr std::string_view kObservationFileMagic = "ROVIOBS1";
constexpr std::string_view kHexDigits = "0123456789abcdef";
constexpr std::uint8_t kNibbleBits = 4;
constexpr std::uint8_t kNibbleMask = 0x0F;

class Writer {
 public:
  void u16(std::uint16_t value) { bits<sizeof(value)>(value); }
  void u32(std::uint32_t value) { bits<sizeof(value)>(value); }
  void u64(std::uint64_t value) { bits<sizeof(value)>(value); }
  void i64(std::int64_t value) { u64(static_cast<std::uint64_t>(value)); }
  void f32(float value) { u32(std::bit_cast<std::uint32_t>(value)); }
  void raw(std::span<const std::uint8_t> data) {
    bytes_.insert(bytes_.end(), data.begin(), data.end());
  }

  bool text(std::string_view value) {
    if (value.size() > std::numeric_limits<std::uint16_t>::max()) {
      return false;
    }
    u16(static_cast<std::uint16_t>(value.size()));
    std::ranges::transform(value, std::back_inserter(bytes_),
                           [](char letter) { return static_cast<std::uint8_t>(letter); });
    return true;
  }

  [[nodiscard]] std::optional<Bytes> finish() && {
    if (bytes_.size() > kMaximumPayload) {
      return std::nullopt;
    }
    return std::move(bytes_);
  }

 private:
  template <std::size_t Count>
  void bits(std::uint64_t value) {
    for (std::size_t index = 0; index < Count; ++index) {
      bytes_.push_back(static_cast<std::uint8_t>(value >> (kByteBits * index)));
    }
  }

  Bytes bytes_;
};

class Reader {
 public:
  explicit Reader(std::span<const std::uint8_t> data) : data_{data} {}

  std::optional<std::uint16_t> u16() {
    const std::optional<std::uint64_t> value = bits(sizeof(std::uint16_t));
    return value ? std::optional{static_cast<std::uint16_t>(*value)} : std::nullopt;
  }
  std::optional<std::uint32_t> u32() {
    const std::optional<std::uint64_t> value = bits(sizeof(std::uint32_t));
    return value ? std::optional{static_cast<std::uint32_t>(*value)} : std::nullopt;
  }
  std::optional<std::uint64_t> u64() { return bits(sizeof(std::uint64_t)); }
  std::optional<std::int64_t> i64() {
    const std::optional<std::uint64_t> value = u64();
    return value ? std::optional{static_cast<std::int64_t>(*value)} : std::nullopt;
  }

  std::optional<std::span<const std::uint8_t>> raw(std::size_t count) {
    if (count > data_.size() - at_) {
      return std::nullopt;
    }
    const std::span<const std::uint8_t> chunk = data_.subspan(at_, count);
    at_ += count;
    return chunk;
  }

  std::optional<std::vector<float>> floats(std::uint32_t count) {
    if (std::uint64_t{count} * sizeof(float) > data_.size() - at_) {
      return std::nullopt;
    }
    std::vector<float> values(count);
    std::ranges::generate(values, [this] { return std::bit_cast<float>(u32().value_or(0)); });
    return values;
  }

  std::optional<std::string> text() {
    const std::optional<std::uint16_t> size = u16();
    const std::optional<std::span<const std::uint8_t>> letters = size ? raw(*size) : std::nullopt;
    if (!letters) {
      return std::nullopt;
    }
    std::string value(letters->size(), '\0');
    std::ranges::transform(*letters, value.begin(),
                           [](std::uint8_t letter) { return static_cast<char>(letter); });
    return value;
  }

  std::optional<Sha256> sha256() {
    const std::optional<std::span<const std::uint8_t>> chunk = raw(kSha256Bytes);
    if (!chunk) {
      return std::nullopt;
    }
    Sha256 digest{};
    std::ranges::copy(*chunk, digest.begin());
    return digest;
  }

  [[nodiscard]] bool done() const { return at_ == data_.size(); }

 private:
  std::optional<std::uint64_t> bits(std::size_t count) {
    const std::optional<std::span<const std::uint8_t>> chunk = raw(count);
    if (!chunk) {
      return std::nullopt;
    }
    std::uint64_t value = 0;
    std::size_t shift = 0;
    for (const std::uint8_t byte : *chunk) {
      value |= std::uint64_t{byte} << shift;
      shift += kByteBits;
    }
    return value;
  }

  std::span<const std::uint8_t> data_;
  std::size_t at_{0};
};

bool image_matches(std::uint32_t width, std::uint32_t height, std::size_t bytes) {
  return std::uint64_t{width} * height * kChannels == bytes;
}

bool put_observation(Writer& writer, const Observation& observation) {
  if (observation.state.size() > std::numeric_limits<std::uint32_t>::max() ||
      !image_matches(observation.width, observation.height, observation.rgb.size())) {
    return false;
  }
  writer.u32(static_cast<std::uint32_t>(observation.state.size()));
  for (const float value : observation.state) {
    writer.f32(value);
  }
  writer.u32(observation.width);
  writer.u32(observation.height);
  writer.raw(observation.rgb);
  return true;
}

std::optional<Observation> get_observation(Reader& reader) {
  const std::optional<std::uint32_t> state_dim = reader.u32();
  std::optional<std::vector<float>> state = state_dim ? reader.floats(*state_dim) : std::nullopt;
  const std::optional<std::uint32_t> width = reader.u32();
  const std::optional<std::uint32_t> height = reader.u32();
  if (!state || !width || !height) {
    return std::nullopt;
  }
  const std::uint64_t pixels = std::uint64_t{*width} * *height * kChannels;
  const std::optional<std::span<const std::uint8_t>> rgb =
      pixels <= kMaximumPayload ? reader.raw(static_cast<std::size_t>(pixels)) : std::nullopt;
  if (!rgb) {
    return std::nullopt;
  }
  return Observation{.state = std::move(*state),
                     .width = *width,
                     .height = *height,
                     .rgb = Bytes(rgb->begin(), rgb->end())};
}

}  // namespace

Bytes encode_header(FrameType type, std::uint32_t length) {
  Writer writer;
  writer.u32(kMagic);
  writer.u16(kVersion);
  writer.u16(static_cast<std::uint16_t>(type));
  writer.u32(length);
  return std::move(writer).finish().value_or(Bytes{});
}

std::optional<Header> decode_header(std::span<const std::uint8_t> bytes) {
  Reader reader{bytes};
  const std::optional<std::uint32_t> magic = reader.u32();
  const std::optional<std::uint16_t> version = reader.u16();
  const std::optional<std::uint16_t> type = reader.u16();
  const std::optional<std::uint32_t> length = reader.u32();
  if (!magic || !version || !type || !length || !reader.done() || *magic != kMagic ||
      *version != kVersion || *length > kMaximumPayload ||
      *type < static_cast<std::uint16_t>(FrameType::kHello) ||
      *type > static_cast<std::uint16_t>(FrameType::kError)) {
    return std::nullopt;
  }
  return Header{.type = static_cast<FrameType>(static_cast<std::uint8_t>(*type)),
                .length = *length};
}

std::optional<Bytes> encode_hello(const Hello& hello) {
  Writer writer;
  writer.raw(hello.policy);
  writer.raw(hello.adapter);
  writer.u32(hello.action_steps);
  writer.u32(hello.action_dim);
  writer.u32(hello.state_dim);
  writer.u32(hello.image_width);
  writer.u32(hello.image_height);
  writer.i64(hello.action_period_ns);
  if (!writer.text(hello.policy_type) || !writer.text(hello.task) ||
      !writer.text(hello.robot_type) || !writer.text(hello.device) || !writer.text(hello.runtime)) {
    return std::nullopt;
  }
  return std::move(writer).finish();
}

std::optional<Bytes> encode_request(const Request& request) {
  Writer writer;
  writer.u64(request.id);
  writer.i64(request.observation_ns);
  if (!put_observation(writer, request.observation)) {
    return std::nullopt;
  }
  return std::move(writer).finish();
}

std::optional<Bytes> encode_response(const Response& response) {
  if (response.actions.size() != std::uint64_t{response.steps} * response.dim) {
    return std::nullopt;
  }
  Writer writer;
  writer.u64(response.id);
  writer.i64(response.observation_ns);
  writer.raw(response.policy);
  writer.u64(response.noise_seed);
  writer.u64(response.inference_ns);
  writer.u64(response.handling_ns);
  writer.u32(response.steps);
  writer.u32(response.dim);
  for (const float value : response.actions) {
    writer.f32(value);
  }
  return std::move(writer).finish();
}

std::optional<Bytes> encode_error(const ErrorReply& error) {
  Writer writer;
  writer.u64(error.id);
  writer.u16(error.code);
  if (!writer.text(error.message)) {
    return std::nullopt;
  }
  return std::move(writer).finish();
}

std::optional<Hello> decode_hello(std::span<const std::uint8_t> payload) {
  Reader reader{payload};
  const std::optional<PolicyId> policy = reader.sha256();
  const std::optional<Sha256> adapter = reader.sha256();
  const std::optional<std::uint32_t> steps = reader.u32();
  const std::optional<std::uint32_t> dim = reader.u32();
  const std::optional<std::uint32_t> state_dim = reader.u32();
  const std::optional<std::uint32_t> width = reader.u32();
  const std::optional<std::uint32_t> height = reader.u32();
  const std::optional<std::int64_t> period = reader.i64();
  std::optional<std::string> type = reader.text();
  std::optional<std::string> task = reader.text();
  std::optional<std::string> robot_type = reader.text();
  std::optional<std::string> device = reader.text();
  std::optional<std::string> runtime = reader.text();
  if (!policy || !adapter || !steps || !dim || !state_dim || !width || !height || !period ||
      !type || !task || !robot_type || !device || !runtime || !reader.done()) {
    return std::nullopt;
  }
  return Hello{.policy = *policy,
               .adapter = *adapter,
               .action_steps = *steps,
               .action_dim = *dim,
               .state_dim = *state_dim,
               .image_width = *width,
               .image_height = *height,
               .action_period_ns = *period,
               .policy_type = std::move(*type),
               .task = std::move(*task),
               .robot_type = std::move(*robot_type),
               .device = std::move(*device),
               .runtime = std::move(*runtime)};
}

std::optional<Request> decode_request(std::span<const std::uint8_t> payload) {
  Reader reader{payload};
  const std::optional<std::uint64_t> id = reader.u64();
  const std::optional<std::int64_t> observation_ns = reader.i64();
  std::optional<Observation> observation =
      id && observation_ns ? get_observation(reader) : std::nullopt;
  if (!observation || !reader.done()) {
    return std::nullopt;
  }
  return Request{
      .id = *id, .observation_ns = *observation_ns, .observation = std::move(*observation)};
}

std::optional<Response> decode_response(std::span<const std::uint8_t> payload) {
  Reader reader{payload};
  const std::optional<std::uint64_t> id = reader.u64();
  const std::optional<std::int64_t> observation_ns = reader.i64();
  const std::optional<PolicyId> policy = reader.sha256();
  const std::optional<std::uint64_t> seed = reader.u64();
  const std::optional<std::uint64_t> inference = reader.u64();
  const std::optional<std::uint64_t> handling = reader.u64();
  const std::optional<std::uint32_t> steps = reader.u32();
  const std::optional<std::uint32_t> dim = reader.u32();
  if (!id || !observation_ns || !policy || !seed || !inference || !handling || !steps || !dim) {
    return std::nullopt;
  }
  const std::uint64_t count = std::uint64_t{*steps} * *dim;
  std::optional<std::vector<float>> actions =
      count <= kMaximumPayload ? reader.floats(static_cast<std::uint32_t>(count)) : std::nullopt;
  if (!actions || !reader.done()) {
    return std::nullopt;
  }
  return Response{.id = *id,
                  .observation_ns = *observation_ns,
                  .policy = *policy,
                  .noise_seed = *seed,
                  .inference_ns = *inference,
                  .handling_ns = *handling,
                  .steps = *steps,
                  .dim = *dim,
                  .actions = std::move(*actions)};
}

std::optional<ErrorReply> decode_error(std::span<const std::uint8_t> payload) {
  Reader reader{payload};
  const std::optional<std::uint64_t> id = reader.u64();
  const std::optional<std::uint16_t> code = reader.u16();
  std::optional<std::string> message = reader.text();
  if (!id || !code || !message || !reader.done()) {
    return std::nullopt;
  }
  return ErrorReply{.id = *id, .code = *code, .message = std::move(*message)};
}

std::size_t request_bytes(const Observation& observation) {
  return kRequestFixedBytes + (sizeof(float) * observation.state.size()) + observation.rgb.size();
}

std::size_t response_bytes(std::uint32_t steps, std::uint32_t dim) {
  return kResponseFixedBytes + (sizeof(float) * std::size_t{steps} * dim);
}

std::optional<RecordedObservation> read_observation_file(const std::filesystem::path& path) {
  std::error_code error;
  const std::uintmax_t size = std::filesystem::file_size(path, error);
  if (error || size > kMaximumPayload) {
    return std::nullopt;
  }
  std::ifstream stream{path, std::ios::binary};
  std::string contents(static_cast<std::size_t>(size), '\0');
  if (!stream.read(contents.data(), static_cast<std::streamsize>(size))) {
    return std::nullopt;
  }
  Bytes bytes(contents.size());
  std::ranges::transform(contents, bytes.begin(),
                         [](char letter) { return static_cast<std::uint8_t>(letter); });
  Reader reader{bytes};
  const std::optional<std::span<const std::uint8_t>> magic =
      reader.raw(kObservationFileMagic.size());
  const auto as_letter = [](std::uint8_t letter) { return static_cast<char>(letter); };
  if (!magic || !std::ranges::equal(*magic, kObservationFileMagic, {}, as_letter)) {
    return std::nullopt;
  }
  const std::optional<std::uint32_t> episode = reader.u32();
  const std::optional<std::uint32_t> frame = reader.u32();
  std::optional<Observation> observation =
      episode && frame ? get_observation(reader) : std::nullopt;
  if (!observation || !reader.done()) {
    return std::nullopt;
  }
  return RecordedObservation{
      .episode = *episode, .frame = *frame, .observation = std::move(*observation)};
}

bool write_observation_file(const std::filesystem::path& path,
                            const RecordedObservation& recorded) {
  Bytes magic(kObservationFileMagic.size());
  std::ranges::transform(kObservationFileMagic, magic.begin(),
                         [](char letter) { return static_cast<std::uint8_t>(letter); });
  Writer writer;
  writer.raw(magic);
  writer.u32(recorded.episode);
  writer.u32(recorded.frame);
  std::optional<Bytes> bytes =
      put_observation(writer, recorded.observation) ? std::move(writer).finish() : std::nullopt;
  if (!bytes) {
    return false;
  }
  std::string contents(bytes->size(), '\0');
  std::ranges::transform(*bytes, contents.begin(),
                         [](std::uint8_t letter) { return static_cast<char>(letter); });
  std::ofstream stream{path, std::ios::binary | std::ios::trunc};
  return static_cast<bool>(
      stream.write(contents.data(), static_cast<std::streamsize>(contents.size())));
}

std::string to_hex(const Sha256& digest) {
  std::string hex;
  hex.reserve(2 * digest.size());
  for (const std::uint8_t byte : digest) {
    hex.push_back(kHexDigits.at(byte >> kNibbleBits));
    hex.push_back(kHexDigits.at(byte & kNibbleMask));
  }
  return hex;
}

std::optional<PolicyId> policy_from_hex(std::string_view hex) {
  PolicyId id{};
  if (hex.size() != 2 * id.size()) {
    return std::nullopt;
  }
  for (std::size_t index = 0; index < id.size(); ++index) {
    const std::size_t high = kHexDigits.find(hex.at(2 * index));
    const std::size_t low = kHexDigits.find(hex.at((2 * index) + 1));
    if (high == std::string_view::npos || low == std::string_view::npos) {
      return std::nullopt;
    }
    id.at(index) = static_cast<std::uint8_t>((high << kNibbleBits) | low);
  }
  return id;
}

}  // namespace rovi::policy
