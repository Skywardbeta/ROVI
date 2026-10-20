// FNV-1a 64-bit digest over little-endian integer words for deterministic run fingerprints.
#ifndef ROVI_CORE_DIGEST_HPP_
#define ROVI_CORE_DIGEST_HPP_

#include <cstdint>

namespace rovi::core {

class Fnv1aDigest {
 public:
  constexpr void add(std::uint64_t value) {
    for (unsigned byte = 0; byte < kBytesPerWord; ++byte) {
      hash_ ^= (value >> (byte * kBitsPerByte)) & kByteMask;
      hash_ *= kPrime;
    }
  }

  constexpr void add(std::int64_t value) { add(static_cast<std::uint64_t>(value)); }

  [[nodiscard]] constexpr std::uint64_t value() const { return hash_; }

 private:
  static constexpr std::uint64_t kOffset = 14'695'981'039'346'656'037ULL;
  static constexpr std::uint64_t kPrime = 1'099'511'628'211ULL;
  static constexpr std::uint64_t kByteMask = 0xFFU;
  static constexpr unsigned kBitsPerByte = 8;
  static constexpr unsigned kBytesPerWord = 8;

  std::uint64_t hash_{kOffset};
};

}  // namespace rovi::core

#endif  // ROVI_CORE_DIGEST_HPP_
