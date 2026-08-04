// 链路健康检测回归测试。
//
// 板子以 ~20 Hz 广播 cmd 0x01 心跳帧。此前 ProtocolV1::DecodeFrame 的
// switch 只处理 0x20~0x24, 心跳被静默丢弃, 于是"串口断了"和"串口通着但
// 关节没动"在上层完全无法区分 —— 力矩模式下这是安全缺口。
//
// 帧字节取自真实抓包 (/dev/ttyUSB0 @ 921600, 3 s, 2480 帧零校验错误),
// 不是手工构造, 以免测试和固件的校验和约定脱节。

#include <gtest/gtest.h>

#include <manipulator/robotics/protocol/protocol_v1.h>

#include <cstdint>
#include <limits>
#include <vector>

namespace {

// 实测心跳帧: FE 55 | cmd=0x01 | len=1 | payload=0x1B | sum=0x70 | add=0x6A
const std::vector<uint8_t> kHeartbeat = {0xFE, 0x55, 0x01, 0x01, 0x1B, 0x70, 0x6A};

void FeedAll(manipulator::protocol::ProtocolV1& p, const std::vector<uint8_t>& bytes) {
  for (uint8_t b : bytes) {
    p.Feed(b);
  }
}

}  // namespace

TEST(ProtocolHeartbeat, NoHeartbeatMeansLinkIsNotAlive) {
  manipulator::protocol::ProtocolV1 p;
  EXPECT_EQ(p.GetHeartbeatCount(), 0u);
  EXPECT_TRUE(std::isinf(p.GetSecondsSinceHeartbeat()));
  EXPECT_FALSE(p.IsLinkAlive(1.0))
      << "a link that has never spoken must never be reported alive";
}

TEST(ProtocolHeartbeat, RealCapturedFrameIsCountedAndMarksLinkAlive) {
  manipulator::protocol::ProtocolV1 p;
  FeedAll(p, kHeartbeat);
  EXPECT_EQ(p.GetHeartbeatCount(), 1u);
  EXPECT_LT(p.GetSecondsSinceHeartbeat(), 1.0);
  EXPECT_TRUE(p.IsLinkAlive(1.0));
}

TEST(ProtocolHeartbeat, HeartbeatCountAccumulates) {
  manipulator::protocol::ProtocolV1 p;
  for (int i = 0; i < 20; ++i) {
    FeedAll(p, kHeartbeat);
  }
  EXPECT_EQ(p.GetHeartbeatCount(), 20u);
}

TEST(ProtocolHeartbeat, CorruptedHeartbeatIsRejected) {
  manipulator::protocol::ProtocolV1 p;
  auto bad = kHeartbeat;
  bad[5] ^= 0xFF;  // break sumCheck
  FeedAll(p, bad);
  EXPECT_EQ(p.GetHeartbeatCount(), 0u)
      << "checksum must gate the heartbeat, otherwise line noise fakes liveness";
}

TEST(ProtocolHeartbeat, HeartbeatIsNotCountedAsUnhandled) {
  manipulator::protocol::ProtocolV1 p;
  FeedAll(p, kHeartbeat);
  EXPECT_EQ(p.GetUnhandledFrameCount(), 0u);
}

TEST(ProtocolHeartbeat, UndocumentedFrameIsCountedNotInterpreted) {
  // 板子还以 ~2 Hz 发 cmd 0x25 (28 B, payload 前三个 int32 为 1),
  // 含义未由固件文档确认。这里只统计, 不解释。
  manipulator::protocol::ProtocolV1 p;
  std::vector<uint8_t> frame = {0xFE, 0x55, 0x25, 0x04, 0x01, 0x00, 0x00, 0x00};
  uint8_t sum = 0, add = 0;
  for (uint8_t b : frame) {
    sum += b;
    add += sum;
  }
  frame.push_back(sum);
  frame.push_back(add);

  FeedAll(p, frame);
  EXPECT_EQ(p.GetUnhandledFrameCount(), 1u);
  EXPECT_EQ(p.GetLastUnhandledCommand(), 0x25);
  EXPECT_EQ(p.GetHeartbeatCount(), 0u);
}
