// SHA-256 against the FIPS 180-2 example messages and the padding boundaries. Every expected
// digest was computed outside this repository (Python's hashlib), never by the code under test.

#include <gtest/gtest.h>

#include <string>

#include "c3d/sha256.hpp"

using c3d::Sha256Hex;

TEST(Sha256, MatchesTheNistExampleMessages) {
  EXPECT_EQ(Sha256Hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(Sha256Hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(Sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(Sha256Hex("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmno"
                      "ijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"),
            "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1");
  EXPECT_EQ(Sha256Hex(std::string(1000000, 'a')),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

// 55 bytes is the longest tail whose length still fits the same block; 56 and 63 need a second
// one, and 64 is a whole block with the padding entirely in the next.
TEST(Sha256, PadsCorrectlyEitherSideOfTheBlockBoundary) {
  EXPECT_EQ(Sha256Hex(std::string(55, 'a')),
            "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318");
  EXPECT_EQ(Sha256Hex(std::string(56, 'a')),
            "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a");
  EXPECT_EQ(Sha256Hex(std::string(63, 'a')),
            "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34");
  EXPECT_EQ(Sha256Hex(std::string(64, 'a')),
            "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb");
}

TEST(Sha256, HashesEveryByteValueAsAnUnsignedOctet) {
  std::string every_byte;
  for (int i = 0; i < 256; ++i) every_byte.push_back(static_cast<char>(i));
  EXPECT_EQ(Sha256Hex(every_byte),
            "40aff2e9d2d8922e47afd4648e6967497158785fbd1da870e7110266bf944880");
  // Bytes 0x80..0xE3: one whole block, then a 36-byte tail made only of high-bit bytes, which is
  // the path the 256-byte case above never takes.
  EXPECT_EQ(Sha256Hex(every_byte.substr(128, 100)),
            "ad712f7800dbbbd6009f071d78f3d3e86f239ab7c483712069034c9b7457996d");
  EXPECT_EQ(Sha256Hex("caf\xC3\xA9"),
            "850f7dc43910ff890f8879c0ed26fe697c93a067ad93a7d50f466a7028a9bf4e");
}
