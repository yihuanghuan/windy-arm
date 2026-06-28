#include <memory>
#include <string>

#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace manipulator {

class FlightStatusBridge : public rclcpp::Node {
 public:
  FlightStatusBridge() : Node("flight_status_bridge") {
    declare_parameter<std::string>("mavros_state_topic", "/mavros/state");
    declare_parameter<std::string>("mavros_battery_topic", "/mavros/battery");
    declare_parameter<std::string>("armed_topic", "/flight/armed");
    declare_parameter<std::string>("battery_topic", "/flight/battery");

    auto mavros_state_topic = get_parameter("mavros_state_topic").as_string();
    auto mavros_battery_topic = get_parameter("mavros_battery_topic").as_string();
    auto armed_topic = get_parameter("armed_topic").as_string();
    auto battery_topic = get_parameter("battery_topic").as_string();

    armed_pub_ = create_publisher<std_msgs::msg::Bool>(armed_topic, 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(battery_topic, 10);

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        mavros_state_topic,
        10,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
          std_msgs::msg::Bool armed_msg;
          armed_msg.data = msg->armed;
          armed_pub_->publish(armed_msg);
        });

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
        mavros_battery_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
          battery_pub_->publish(*msg);
        });
  }

 private:
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
};

}  // namespace manipulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::FlightStatusBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
