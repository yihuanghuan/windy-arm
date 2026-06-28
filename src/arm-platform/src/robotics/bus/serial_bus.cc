#include <manipulator/robotics/bus/serial_bus.h>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <iostream>

namespace manipulator::bus {

SerialBus::SerialBus(const std::string& port_name, uint32_t baudrate) 
 : AbsBus() {
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    serial_.setPort(port_name);
    serial_.setBaudrate(baudrate);
    serial_.setTimeout(to);

    try {
        serial_.open();
    } catch (serial::IOException& e) {
        throw std::runtime_error("Unable to open serial port " + port_name);
    }

}

SerialBus::~SerialBus() {
  if (serial_.isOpen()) serial_.close();
}

void SerialBus::SendCore(const std::vector<uint8_t>& data) {
  if (serial_.isOpen()) {
    serial_.write(data.data(), data.size());
  }
}

void SerialBus::ReadCore(std::vector<uint8_t>& data) {
    size_t avail = serial_.available();
    // std::vector<uint8_t> buf(avail);
    data.resize(avail);
    // serial_.read(buf.data(), avail);
    serial_.read(data.data(), avail);
    // return buf;
}

}  // namespace arm_platform
