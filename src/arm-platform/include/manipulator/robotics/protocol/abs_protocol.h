#include <manipulator/robotics/protocol/i_protocol.h>
#include <manipulator/robotics/motor/i_motor.h>

namespace manipulator::protocol {

class AbsProtocol : public IProtocol {
 public:
  virtual void Feed(const uint8_t byte) = 0;
  virtual void Pop(std::vector<uint8_t>& stream) = 0;

  // Observer pattern methods
  void Attach(motor::IMotor::SharedPtr observer);
  void Detach(motor::IMotor::SharedPtr observer);
 
 protected:
  void Notify();

 private:
  std::vector<motor::IMotor::SharedPtr> observers_;

};
  
} // namespace name
