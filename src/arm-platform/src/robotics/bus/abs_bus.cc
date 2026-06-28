#include <manipulator/robotics/bus/abs_bus.h>

namespace manipulator::bus {
AbsBus::AbsBus() {
  
}

void AbsBus::Send() {
  std::vector<uint8_t> bytes;
  protocol_->Pop(bytes);
  SendCore(bytes);
}

void AbsBus::Read() {
  std::vector<uint8_t> bytes;
  ReadCore(bytes);
  if (protocol_) {
    for (auto byte : bytes) protocol_->Feed(byte);
  }
  // protocol_factory_->IterateEach([this, &bytes](const auto& name, auto& protocol) {
  //   for(auto byte : bytes) protocol->Feed(byte); //TODO: this is not right when there are multiple protocols
  // });
}

void AbsBus::SetProtocol(protocol::IProtocol::SharedPtr protocol) {
  protocol_ = protocol;
}

}