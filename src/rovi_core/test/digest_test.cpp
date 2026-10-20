#include "rovi_core/digest.hpp"

#include <cstdint>
#include <initializer_list>

#include <gtest/gtest.h>

namespace rovi::core {
namespace {

std::uint64_t digest_of(std::initializer_list<std::int64_t> words) {
  Fnv1aDigest digest;
  for (const std::int64_t word : words) {
    digest.add(word);
  }
  return digest.value();
}

TEST(Fnv1aDigest, MatchesReferenceVectorsOverLittleEndianWords) {
  EXPECT_EQ(Fnv1aDigest{}.value(), 14'695'981'039'346'656'037ULL);
  EXPECT_EQ(digest_of({0}), 12'161'962'213'042'174'405ULL);
  EXPECT_EQ(digest_of({1}), 9'929'646'806'074'584'996ULL);
  EXPECT_EQ(digest_of({-1}), 10'157'053'723'145'373'757ULL);
  EXPECT_EQ(digest_of({1, 2}), 8'581'494'755'304'202'342ULL);
}

TEST(Fnv1aDigest, SignedAndUnsignedWordsWithEqualBitsAgree) {
  Fnv1aDigest unsigned_digest;
  unsigned_digest.add(std::uint64_t{0xFFFF'FFFF'FFFF'FFFFULL});

  EXPECT_EQ(unsigned_digest.value(), digest_of({-1}));
}

}  // namespace
}  // namespace rovi::core
