#pragma once

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace manipulator {

class RobotHealthAggregator : public rclcpp::Node {
 public:
  RobotHealthAggregator();

 private:
  struct ModuleState {
    std::string topic;
    std::string status;
    std::string code;
    std::string message;
    std::string raw;
    rclcpp::Time last_seen;
    bool received = false;
  };

  static std::string ModuleFromTopic(const std::string& topic);
  static std::string ExtractJsonString(const std::string& data, const std::string& key, const std::string& fallback);
  static std::string ExtractJsonObject(const std::string& data, const std::string& key);
  static std::string EscapeJson(const std::string& value);
  static int StatusRank(const std::string& status);
  static std::string RankStatus(int rank);

  void PublishHealth();

  double timeout_sec_ = 3.0;
  std::vector<std::string> health_topics_;
  std::map<std::string, ModuleState> states_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_health_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace manipulator
