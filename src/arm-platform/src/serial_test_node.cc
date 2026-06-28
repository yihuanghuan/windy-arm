#include <serial/serial.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

constexpr uint8_t kHeader0 = 0x55;
constexpr uint8_t kHeader1 = 0xAA;
constexpr size_t kHeaderSize = 2;
constexpr size_t kSeqSize = 4;
constexpr size_t kLenSize = 2;
constexpr size_t kChecksumSize = 2;
constexpr size_t kFrameOverhead = kHeaderSize + kSeqSize + kLenSize + kChecksumSize;

struct Stats {
  uint64_t rx_bytes = 0;
  uint64_t tx_frames = 0;
  uint64_t rx_frames = 0;
  uint64_t bad_checksum = 0;
  uint64_t payload_errors = 0;
  uint64_t lost_frames = 0;
  uint64_t out_of_order = 0;
  uint64_t dropped_bytes = 0;
  uint64_t bad_header = 0;
  uint64_t bad_length = 0;
  uint64_t raw_samples = 0;
  uint32_t expected_seq = 0;
  bool has_expected_seq = false;
};

void PutU16(std::vector<uint8_t>& data, uint16_t value) {
  data.push_back(static_cast<uint8_t>(value & 0xFF));
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void PutU32(std::vector<uint8_t>& data, uint32_t value) {
  data.push_back(static_cast<uint8_t>(value & 0xFF));
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

uint16_t GetU16(const std::vector<uint8_t>& data, size_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(data[offset + 1] << 8);
}

uint32_t GetU32(const std::vector<uint8_t>& data, size_t offset) {
  return static_cast<uint32_t>(data[offset]) |
         static_cast<uint32_t>(data[offset + 1] << 8) |
         static_cast<uint32_t>(data[offset + 2] << 16) |
         static_cast<uint32_t>(data[offset + 3] << 24);
}

uint16_t Checksum(const std::vector<uint8_t>& data, size_t begin, size_t end) {
  uint32_t sum = 0;
  for (size_t i = begin; i < end; ++i) {
    sum += data[i];
  }
  return static_cast<uint16_t>(sum & 0xFFFF);
}

std::string ToHex(const std::vector<uint8_t>& data, size_t max_size = 64) {
  std::ostringstream stream;
  size_t size = std::min(data.size(), max_size);
  for (size_t i = 0; i < size; ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
           << static_cast<int>(data[i]);
  }
  if (data.size() > max_size) {
    stream << " ...";
  }
  return stream.str();
}

std::vector<uint8_t> MakeFrame(uint32_t seq, uint16_t payload_size) {
  std::vector<uint8_t> frame;
  frame.reserve(kFrameOverhead + payload_size);
  frame.push_back(kHeader0);
  frame.push_back(kHeader1);
  PutU32(frame, seq);
  PutU16(frame, payload_size);

  for (uint16_t i = 0; i < payload_size; ++i) {
    frame.push_back(static_cast<uint8_t>((seq + i) & 0xFF));
  }

  PutU16(frame, Checksum(frame, 0, frame.size()));
  return frame;
}

bool CheckPayload(uint32_t seq, const std::vector<uint8_t>& frame, size_t payload_offset, uint16_t payload_size,
                  size_t& error_index, uint8_t& expected, uint8_t& actual) {
  for (uint16_t i = 0; i < payload_size; ++i) {
    expected = static_cast<uint8_t>((seq + i) & 0xFF);
    actual = frame[payload_offset + i];
    if (actual != expected) {
      error_index = i;
      return false;
    }
  }
  return true;
}

void PrintUsage(const char* program) {
  std::cout << "usage:\n"
            << "  " << program << " tx <port> <baud> [payload_size] [interval_ms]\n"
            << "  " << program << " rx <port> <baud> [payload_size]\n"
            << "example:\n"
            << "  " << program << " tx /dev/ttyTHS0 921600 64 10\n"
            << "  " << program << " rx /dev/ttyTHS0 921600 64\n";
}

void OpenSerial(serial::Serial& serial_port, const std::string& port, uint32_t baudrate) {
  serial_port.setPort(port);
  serial_port.setBaudrate(baudrate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(20);
  serial_port.setTimeout(timeout);
  serial_port.open();

  if (!serial_port.isOpen()) {
    throw std::runtime_error("open serial port failed: " + port);
  }
}

void RunTx(serial::Serial& serial_port, uint16_t payload_size, int interval_ms) {
  Stats stats;
  auto last_print = std::chrono::steady_clock::now();

  for (uint32_t seq = 0;; ++seq) {
    auto frame = MakeFrame(seq, payload_size);
    serial_port.write(frame.data(), frame.size());
    ++stats.tx_frames;

    auto now = std::chrono::steady_clock::now();
    if (now - last_print >= std::chrono::seconds(1)) {
      std::cout << "tx_frames=" << stats.tx_frames
                << " seq=" << seq
                << " frame_bytes=" << frame.size()
                << " sample=" << ToHex(frame, 24) << std::endl;
      last_print = now;
    }

    if (interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
  }
}

void HandleFrame(const std::vector<uint8_t>& frame, Stats& stats) {
  uint32_t seq = GetU32(frame, 2);
  uint16_t payload_size = GetU16(frame, 6);
  uint16_t expected_checksum = GetU16(frame, frame.size() - 2);
  uint16_t actual_checksum = Checksum(frame, 0, frame.size() - 2);

  if (expected_checksum != actual_checksum) {
    ++stats.bad_checksum;
    std::cout << "bad_checksum seq=" << seq
              << " expected=0x" << std::hex << expected_checksum
              << " actual=0x" << actual_checksum << std::dec
              << " frame=" << ToHex(frame) << std::endl;
    return;
  }

  if (stats.has_expected_seq) {
    if (seq > stats.expected_seq) {
      stats.lost_frames += seq - stats.expected_seq;
    } else if (seq < stats.expected_seq) {
      ++stats.out_of_order;
    }
  } else {
    stats.has_expected_seq = true;
  }
  stats.expected_seq = seq + 1;

  size_t error_index = 0;
  uint8_t expected = 0;
  uint8_t actual = 0;
  if (!CheckPayload(seq, frame, 8, payload_size, error_index, expected, actual)) {
    ++stats.payload_errors;
    std::cout << "payload_error seq=" << seq
              << " index=" << error_index
              << " expected=0x" << std::hex << static_cast<int>(expected)
              << " actual=0x" << static_cast<int>(actual) << std::dec
              << " frame=" << ToHex(frame) << std::endl;
    return;
  }

  ++stats.rx_frames;
}

void RunRx(serial::Serial& serial_port, uint16_t max_payload_size) {
  Stats stats;
  std::vector<uint8_t> buffer;
  auto last_print = std::chrono::steady_clock::now();

  for (;;) {
    size_t available = serial_port.available();
    if (available > 0) {
      std::vector<uint8_t> chunk;
      serial_port.read(chunk, available);
      stats.rx_bytes += chunk.size();
      if (stats.raw_samples < 5) {
        std::cout << "raw_sample bytes=" << chunk.size()
                  << " data=" << ToHex(chunk, 80) << std::endl;
        ++stats.raw_samples;
      }
      buffer.insert(buffer.end(), chunk.begin(), chunk.end());
    }

    while (buffer.size() >= kFrameOverhead) {
      if (buffer[0] != kHeader0 || buffer[1] != kHeader1) {
        buffer.erase(buffer.begin());
        ++stats.dropped_bytes;
        ++stats.bad_header;
        continue;
      }

      uint16_t payload_size = GetU16(buffer, 6);
      if (payload_size > max_payload_size) {
        std::vector<uint8_t> sample(buffer.begin(), buffer.begin() + std::min<size_t>(buffer.size(), 16));
        std::cout << "bad_length payload_size=" << payload_size
                  << " max_payload_size=" << max_payload_size
                  << " head=" << ToHex(sample) << std::endl;
        buffer.erase(buffer.begin());
        ++stats.dropped_bytes;
        ++stats.bad_length;
        continue;
      }

      size_t frame_size = kFrameOverhead + payload_size;
      if (buffer.size() < frame_size) {
        break;
      }

      std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + frame_size);
      buffer.erase(buffer.begin(), buffer.begin() + frame_size);
      HandleFrame(frame, stats);
    }

    auto now = std::chrono::steady_clock::now();
    if (now - last_print >= std::chrono::seconds(1)) {
      std::cout << "rx_bytes=" << stats.rx_bytes
                << " rx_frames=" << stats.rx_frames
                << " bad_checksum=" << stats.bad_checksum
                << " payload_errors=" << stats.payload_errors
                << " lost_frames=" << stats.lost_frames
                << " out_of_order=" << stats.out_of_order
                << " dropped_bytes=" << stats.dropped_bytes
                << " bad_header=" << stats.bad_header
                << " bad_length=" << stats.bad_length
                << " buffered=" << buffer.size() << std::endl;
      last_print = now;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

int main(int argc, char** argv) {
  if (argc < 4) {
    PrintUsage(argv[0]);
    return 1;
  }

  std::string mode = argv[1];
  std::string port = argv[2];
  uint32_t baudrate = static_cast<uint32_t>(std::stoul(argv[3]));
  uint16_t payload_size = argc > 4 ? static_cast<uint16_t>(std::stoul(argv[4])) : 64;
  int interval_ms = argc > 5 ? std::stoi(argv[5]) : 10;

  if (payload_size > 4096) {
    std::cerr << "payload_size must be <= 4096" << std::endl;
    return 1;
  }

  try {
    serial::Serial serial_port;
    OpenSerial(serial_port, port, baudrate);
    std::cout << "opened " << port << " @ " << baudrate << " mode=" << mode
              << " payload_size=" << payload_size << std::endl;

    if (mode == "tx") {
      RunTx(serial_port, payload_size, interval_ms);
    } else if (mode == "rx") {
      RunRx(serial_port, payload_size);
    } else {
      PrintUsage(argv[0]);
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "serial test failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
