#include <manipulator/robotics/protocol/abs_protocol.h>

namespace manipulator::protocol { 
void AbsProtocol::Attach(motor::IMotor::SharedPtr observer) {
  observers_.push_back(observer);
}

void AbsProtocol::Detach(motor::IMotor::SharedPtr observer) {
  observers_.erase(std::remove(observers_.begin(), observers_.end(), observer),
                   observers_.end());
}

void AbsProtocol::Notify() {
  for (auto observer : observers_) {
    observer->UpdateState();
  }
}
}
