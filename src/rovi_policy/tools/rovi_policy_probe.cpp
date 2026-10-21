// Sends recorded observations straight to a policy server and reports validated chunks and timing.
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "rovi_policy/chunk_check.hpp"
#include "rovi_policy/policy_client.hpp"
#include "rovi_policy/wire.hpp"

namespace {

using rovi::policy::CallResult;
using rovi::policy::CallStatus;

constexpr int kUsageError = 2;
constexpr double kNsPerMs = 1e6;
constexpr std::chrono::milliseconds kDefaultConnect{5'000};
constexpr std::chrono::milliseconds kDefaultReply{10'000};
constexpr std::size_t kPercentile = 95;
constexpr std::size_t kHundred = 100;

struct Options {
  rovi::policy::Endpoint endpoint;
  rovi::policy::PolicyId policy{};
  std::optional<std::string> task;
  std::vector<std::string> observations;
  std::uint64_t repeat{1};
  rovi::policy::ClientTiming timing{.connect = kDefaultConnect, .reply = kDefaultReply};
};

template <typename Number>
std::optional<Number> number(std::string_view text) {
  Number value{};
  const auto [end, error] = std::from_chars(text.begin(), text.end(), value);
  return error == std::errc{} && end == text.end() ? std::optional{value} : std::nullopt;
}

std::optional<Options> parse(const std::vector<std::string_view>& arguments) {
  Options options;
  bool has_policy = false;
  for (std::size_t index = 1; index + 1 < arguments.size(); index += 2) {
    const std::string_view flag = arguments.at(index);
    const std::string_view value = arguments.at(index + 1);
    if (flag == "--host") {
      options.endpoint.host = value;
    } else if (flag == "--port" && number<std::uint16_t>(value)) {
      options.endpoint.port = number<std::uint16_t>(value).value_or(0);
    } else if (flag == "--policy" && rovi::policy::policy_from_hex(value)) {
      options.policy = rovi::policy::policy_from_hex(value).value_or(rovi::policy::PolicyId{});
      has_policy = true;
    } else if (flag == "--task") {
      options.task = std::string{value};
    } else if (flag == "--observation") {
      options.observations.emplace_back(value);
    } else if (flag == "--repeat" && number<std::uint64_t>(value).value_or(0) > 0) {
      options.repeat = number<std::uint64_t>(value).value_or(1);
    } else if (flag == "--reply-ms" && number<std::int64_t>(value).value_or(0) > 0) {
      options.timing.reply = std::chrono::milliseconds{number<std::int64_t>(value).value_or(0)};
    } else {
      return std::nullopt;
    }
  }
  const bool complete = arguments.size() % 2 == 1 && !options.endpoint.host.empty() &&
                        options.endpoint.port != 0 && has_policy && !options.observations.empty();
  return complete ? std::optional{options} : std::nullopt;
}

double milliseconds(std::int64_t ns) { return static_cast<double>(ns) / kNsPerMs; }

void summarize(std::string_view label, std::vector<std::int64_t> samples) {
  if (samples.empty()) {
    return;
  }
  std::ranges::sort(samples);
  const std::size_t p95 = std::min(samples.size() - 1, (samples.size() * kPercentile) / kHundred);
  std::cout << label << " ms: median " << milliseconds(samples.at(samples.size() / 2)) << ", p95 "
            << milliseconds(samples.at(p95)) << ", max " << milliseconds(samples.back()) << " (n "
            << samples.size() << ")\n";
}

void print_first_action(const rovi::policy::Response& response) {
  std::cout << "[";
  for (std::size_t joint = 0; joint < response.dim && joint < response.actions.size(); ++joint) {
    std::cout << (joint == 0 ? "" : ", ") << response.actions.at(joint);
  }
  std::cout << "]";
}

}  // namespace

int main(int argc, char** argv) {
  const std::optional<Options> options =
      parse(std::vector<std::string_view>(argv, std::next(argv, argc)));
  if (!options) {
    std::cerr << "usage: rovi_policy_probe --host h --port p --policy sha256 [--task text]"
                 " --observation file [--observation file ...] [--repeat n] [--reply-ms t]\n";
    return kUsageError;
  }
  std::vector<rovi::policy::Observation> observations;
  for (const std::string& path : options->observations) {
    std::optional<rovi::policy::RecordedObservation> recorded =
        rovi::policy::read_observation_file(path);
    if (!recorded) {
      std::cerr << "rovi_policy_probe: cannot read " << path << "\n";
      return 1;
    }
    observations.push_back(std::move(recorded->observation));
  }
  rovi::policy::PolicyClient client{
      options->endpoint, {.policy = options->policy, .task = options->task}, options->timing};
  const std::stop_source never;
  std::vector<std::int64_t> round_trips;
  std::vector<std::int64_t> inferences;
  int failures = 0;
  std::cout << std::fixed << std::setprecision(1);
  for (std::uint64_t round = 0; round < options->repeat; ++round) {
    for (std::size_t index = 0; index < observations.size(); ++index) {
      const std::uint64_t id = (round * observations.size()) + index + 1;
      const rovi::policy::Request request{.id = id,
                                          .observation_ns = static_cast<std::int64_t>(id),
                                          .observation = observations.at(index)};
      const auto start = std::chrono::steady_clock::now();
      const CallResult result = client.infer(request, never.get_token());
      const std::int64_t round_trip = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now() - start)
                                          .count();
      if (result.status != CallStatus::kOk || !result.response || !client.hello()) {
        std::cout << "request " << id << ": " << rovi::policy::status_name(result.status) << " ("
                  << result.detail << ")\n";
        ++failures;
        continue;
      }
      const rovi::policy::Response& response = *result.response;
      const rovi::policy::Hello& hello = *client.hello();
      const std::optional<rovi::policy::ChunkFault> fault = rovi::policy::check_chunk(
          response.actions, {.steps = response.steps, .dim = response.dim},
          {.steps = hello.action_steps, .dim = hello.action_dim}, nullptr);
      if (round == 0 && index == 0) {
        std::cout << "policy " << hello.policy_type << " " << rovi::policy::to_hex(hello.policy)
                  << ": " << hello.action_steps << " x " << hello.action_dim << " actions every "
                  << milliseconds(hello.action_period_ns) << " ms\n"
                  << "server adapter " << rovi::policy::to_hex(hello.adapter) << ", task \""
                  << hello.task << "\", robot " << hello.robot_type << ", device " << hello.device
                  << ", runtime " << hello.runtime << "\n";
      }
      std::cout << "request " << id << " (" << options->observations.at(index)
                << "): " << (fault ? rovi::policy::fault_name(*fault) : "valid") << ", round trip "
                << milliseconds(round_trip) << " ms, server inference "
                << milliseconds(static_cast<std::int64_t>(response.inference_ns))
                << " ms, first action ";
      print_first_action(response);
      std::cout << "\n";
      failures += fault ? 1 : 0;
      round_trips.push_back(round_trip);
      inferences.push_back(static_cast<std::int64_t>(response.inference_ns));
    }
  }
  summarize("round trip", round_trips);
  summarize("server inference", inferences);
  return failures == 0 ? 0 : 1;
}
