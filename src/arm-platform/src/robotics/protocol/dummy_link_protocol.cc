#include <arm_platform/protocol/dummy_link_protocol.h>
#include <cstring>

namespace arm_platform::protocol {

DummyLinkProtocol::DummyLinkProtocol() {
    last_state_.position.resize(7);
    last_state_.velocity.resize(7);
    last_state_.current.resize(7);
    last_state_.voltage.resize(7);
    last_state_.temperature.resize(7);
}

/* --------------------- TX --------------------- */

static std::vector<uint8_t> MakeFrame(uint8_t cmd, const void* data, size_t len) {
    std::vector<uint8_t> buf;
    buf.resize(4 + len + 2);

    buf[0] = 0xFE;
    buf[1] = 0xAA;   // send addr
    buf[2] = cmd;
    buf[3] = len;

    if (len > 0)
        std::memcpy(&buf[4], data, len);

    uint8_t sum = 0, add = 0;
    for (size_t i = 0; i < 4 + len; i++) {
        sum += buf[i];
        add += sum;
    }

    buf[4 + len] = sum;
    buf[5 + len] = add;
    return buf;
}

std::vector<uint8_t> DummyLinkProtocol::MakeCmd(const std::vector<double>& data, uint8_t cmd_id) {
    switch (cmd_id) {
        case static_cast<uint8_t>(WriteCmdId::kPositionSet): return MakePositionCmd(data);
        case static_cast<uint8_t>(WriteCmdId::kVelocitySet): return MakeVelocityCmd(data);
        case static_cast<uint8_t>(WriteCmdId::kCurrentSet): return MakeCurrentCmd(data);
        case static_cast<uint8_t>(WriteCmdId::kPSet): return MakePCmd(data);
        case static_cast<uint8_t>(WriteCmdId::kDSet): return MakeDCmd(data);
        case static_cast<uint8_t>(WriteCmdId::kHeartbeat): return MakeHeartbeatCmd();
        default: return {};
    }
}

std::vector<uint8_t> DummyLinkProtocol::MakePositionCmd(const std::vector<double>& pos) {
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(pos[i]);
    buf[1] *= -1;
    return MakeFrame(0x10, buf, 28);
}

std::vector<uint8_t> DummyLinkProtocol::MakeVelocityCmd(const std::vector<double>& vel) {
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(vel[i]);
    buf[1] *= -1;
    return MakeFrame(0x11, buf, 28);
}

std::vector<uint8_t> DummyLinkProtocol::MakeCurrentCmd(const std::vector<double>& cur) {
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(cur[i]);
    buf[1] *= -1;
    return MakeFrame(0x12, buf, 28);
}

std::vector<uint8_t> DummyLinkProtocol::MakePCmd(const std::vector<double>& p) {
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(p[i]);
    return MakeFrame(0x13, buf, 28);
}

std::vector<uint8_t> DummyLinkProtocol::MakeDCmd(const std::vector<double>& d) {
    float buf[7];
    for (int i = 0; i < 7; i++) buf[i] = static_cast<float>(d[i]);
    return MakeFrame(0x14, buf, 28);
}

std::vector<uint8_t> DummyLinkProtocol::MakeHeartbeatCmd() {
    uint8_t buf = 0;
    return MakeFrame(0x01, &buf, 1);
}

/* --------------------- RX FSM --------------------- */

bool DummyLinkProtocol::Feed(uint8_t byte) {
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
        return true;
    }
    return false;
}

bool DummyLinkProtocol::HasFrame() const {
    return has_new_frame_;
}

const JointStateArray& DummyLinkProtocol::PopFrame() {
    has_new_frame_ = false;
    return last_state_;
}

/* --------------------- Decode --------------------- */

void DummyLinkProtocol::Check(LinkFrame& f) {
    uint8_t sum = 0, add = 0;
    uint8_t* p = reinterpret_cast<uint8_t*>(&f);

    for (size_t i = 0; i < 4 + f.dataLen; i++) {
        sum += p[i];
        add += sum;
    }
    f.sumCheck = sum;
    f.addCheck = add;
}

void DummyLinkProtocol::DecodeFrame() {
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
}

template<typename T>
static void DecodeArray(const uint8_t* buf, std::vector<double>& out, int n) {
    for (int i = 0; i < n; i++) {
        T v;
        std::memcpy(&v, buf + i * sizeof(T), sizeof(T));
        out[i] = static_cast<double>(v);
    }
}

void DummyLinkProtocol::DecodePosition() {
    DecodeArray<float>(recv_.dataBuf, last_state_.position, 7);
}

void DummyLinkProtocol::DecodeVelocity() {
    DecodeArray<float>(recv_.dataBuf, last_state_.velocity, 7);
}

void DummyLinkProtocol::DecodeCurrent() {
    DecodeArray<float>(recv_.dataBuf, last_state_.current, 7);
}

void DummyLinkProtocol::DecodeVoltage() {
    DecodeArray<float>(recv_.dataBuf, last_state_.voltage, 7);
}

void DummyLinkProtocol::DecodeTemperature() {
    for (int i = 0; i < 7; i++) {
        uint16_t v;
        std::memcpy(&v, recv_.dataBuf + i * 2, 2);
        last_state_.temperature[i] = v;
    }
}

void DummyLinkProtocol::DecodeHeartbeat() {
    heartbeat_count_ = 0;
    if (connect_flag_ == 0) {
        connect_flag_ = 1;
        // RCLCPP_INFO(this->get_logger(), "connect recovery");
    }
}

} // namespace
