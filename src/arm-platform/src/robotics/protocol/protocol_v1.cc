#include <manipulator/robotics/protocol/protocol_v1.h>
#include <cstring>

namespace manipulator::protocol {

ProtocolV1::ProtocolV1() : positions_(7), velocities_(7), currents_(7), 
  temperatures_(7), voltages_(7), desired_positions_(7), desired_velocities_(7),
  desired_currents_(7), desired_kps_(7), desired_kds_(7) {

}

void ProtocolV1::Pop(std::vector<uint8_t>& out) {

  if (pos_cmd_updated_) {
    // std::vector<uint8_t> data;
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(desired_positions_[i]);
    MakeFrame(0x10, buf, 28, out);
    // out.insert(out.end(), data.begin(), data.end());
    pos_cmd_updated_ = false;
  } 
  
  if (vel_cmd_updated_) {
    float buf[7];
    std::vector<uint8_t> data;
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(desired_velocities_[i]);
    MakeFrame(0x11, buf, 28, data);
    out.insert(out.end(), data.begin(), data.end());
    vel_cmd_updated_ = false;
  } 
  
  if (cur_cmd_updated_) {
    std::vector<uint8_t> data;
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(desired_currents_[i]);
    MakeFrame(0x12, buf, 28, data);
    out.insert(out.end(), data.begin(), data.end());
    cur_cmd_updated_ = false;
  } 
  
  if (kp_cmd_updated_) {
    std::vector<uint8_t> data;
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(desired_kps_[i]);
    MakeFrame(0x13, buf, 28, data);
    out.insert(out.end(), data.begin(), data.end());
    kp_cmd_updated_ = false;
  }
  
  if (kd_cmd_updated_) {
    std::vector<uint8_t> data;
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(desired_kds_[i]);
    MakeFrame(0x14, buf, 28, data);
    out.insert(out.end(), data.begin(), data.end());
    kd_cmd_updated_ = false;
  }
}

void ProtocolV1::Feed(const uint8_t byte) {
  uint8_t* raw = reinterpret_cast<uint8_t*>(&recv_);
  if (recv_step_ < 2) {
    if (byte == raw[recv_step_]) {
      recv_step_++;
    } else {
      recv_step_ = 0;
    }
  }
  else if (recv_step_ == 2) {
    recv_.cmd = byte;
    recv_step_++;
  }
  else if (recv_step_ == 3) {
    recv_.dataLen = byte;
    recv_step_++;
  }
  else if (recv_step_ < 4 + recv_.dataLen) {
    recv_.dataBuf[recv_step_ - 4] = byte;
    recv_step_++;
  }
  else if (recv_step_ == 4 + recv_.dataLen) {
    recv_.sumCheck = byte;
    recv_step_++;
  }
  else if (recv_step_ == 5 + recv_.dataLen) {
    recv_.addCheck = byte;
    DecodeFrame();
    recv_step_ = 0;
  }
}

/* --------------------- Decode --------------------- */

void ProtocolV1::Check(LinkFrame& f) {
  uint8_t sum = 0, add = 0;
  uint8_t* p = reinterpret_cast<uint8_t*>(&f);

  for (size_t i = 0; i < 4 + f.dataLen; i++) {
    sum += p[i];
    add += sum;
  }
  f.sumCheck = sum;
  f.addCheck = add;
}

void ProtocolV1::DecodeFrame() {

  uint8_t sum = recv_.sumCheck;
  uint8_t add = recv_.addCheck;
  Check(recv_);

  if (sum != recv_.sumCheck || add != recv_.addCheck)
    return;

  switch (recv_.cmd) {
    case 0x20: DecodePosition(); break;
    case 0x21: DecodeVelocity(); break;
    case 0x22: DecodeCurrent(); break;
    case 0x23: DecodeVoltage(); break;
    case 0x24: DecodeTemperature(); break;
  }

  Notify();
}

template<typename T>
static void DecodeArray(const uint8_t* buf, std::vector<double>& out, int n) {
  for (int i = 0; i < n; i++) {
    T v;
    std::memcpy(&v, buf + i * sizeof(T), sizeof(T));
    out[i] = static_cast<double>(v);
  }
}

void ProtocolV1::DecodePosition() {
  DecodeArray<float>(recv_.dataBuf, positions_, 7);
}

void ProtocolV1::DecodeVelocity() {
  DecodeArray<float>(recv_.dataBuf, velocities_, 7);
}

void ProtocolV1::DecodeCurrent() {
  DecodeArray<float>(recv_.dataBuf, currents_, 7);
}

void ProtocolV1::DecodeVoltage() {
  DecodeArray<float>(recv_.dataBuf, voltages_, 7);
}

void ProtocolV1::DecodeTemperature() {
  for (int i = 0; i < 7; i++) {
    uint16_t v;
    std::memcpy(&v, recv_.dataBuf + i * 2, 2);
    temperatures_[i] = v;
  }
}

void ProtocolV1::MakeFrame(uint8_t cmd, const void* data, 
                           size_t len, std::vector<uint8_t>& out) {
  out.resize(4 + len + 2);

  out[0] = 0xFE;
  out[1] = 0xAA;   // send addr
  out[2] = cmd;
  out[3] = len;

  if (len > 0)
    std::memcpy(&out[4], data, len);

  uint8_t sum = 0, add = 0;
  for (size_t i = 0; i < 4 + len; i++) {
    sum += out[i];
    add += sum;
  }

  out[4 + len] = sum;
  out[5 + len] = add;
}
}