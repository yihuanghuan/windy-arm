#include <manipulator/ros_nodes/robot_health_aggregator.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>

namespace manipulator {

RobotHealthAggregator::RobotHealthAggregator() : Node("robot_health_aggregator") {
  declare_parameter<std::vector<std::string>>("health_topics", {
      "/health/master_arm",
      "/health/slave_arm",
      "/health/lidar",
      "/health/localization"});
  declare_parameter<double>("timeout_sec", 3.0);
  declare_parameter<double>("publish_rate", 1.0);

  health_topics_ = get_parameter("health_topics").as_string_array();
  timeout_sec_ = get_parameter("timeout_sec").as_double();
  double publish_rate = get_parameter("publish_rate").as_double();
  if (publish_rate <= 0.0) {
    publish_rate = 1.0;
  }

  pub_health_ = create_publisher<std_msgs::msg::String>("/health/robot", 10);

  for (const auto& topic : health_topics_) {
    auto module = ModuleFromTopic(topic);
    states_[module].topic = topic;
    states_[module].status = "NO_DATA";
    states_[module].code = "NO_DATA";
    states_[module].message = "no health received";
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        topic,
        10,
        [this, module](const std_msgs::msg::String::SharedPtr msg) {
          auto& state = states_[module];
          state.last_seen = now();
          state.raw = msg->data;
          state.status = ExtractJsonString(msg->data, "status", "OK");
          state.code = ExtractJsonString(msg->data, "code", "OK");
          state.message = ExtractJsonString(msg->data, "message", "running");
          state.received = true;
        }));
  }

  auto period = std::chrono::duration<double>(1.0 / publish_rate);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period), [this]() {
    PublishHealth();
  });
}

std::string RobotHealthAggregator::ModuleFromTopic(const std::string& topic) {
  auto pos = topic.find_last_of('/');
  if (pos == std::string::npos || pos + 1 >= topic.size()) {
    return topic;
  }
  return topic.substr(pos + 1);
}

std::string RobotHealthAggregator::ExtractJsonString(
    const std::string& data,
    const std::string& key,
    const std::string& fallback) {
  std::string token = "\"" + key + "\"";
  auto key_pos = data.find(token);
  if (key_pos == std::string::npos) {
    return fallback;
  }
  auto colon_pos = data.find(':', key_pos + token.size());
  if (colon_pos == std::string::npos) {
    return fallback;
  }
  auto first_quote = data.find('"', colon_pos + 1);
  if (first_quote == std::string::npos) {
    return fallback;
  }
  auto second_quote = data.find('"', first_quote + 1);
  if (second_quote == std::string::npos) {
    return fallback;
  }
  return data.substr(first_quote + 1, second_quote - first_quote - 1);
}

std::string RobotHealthAggregator::ExtractJsonObject(const std::string& data, const std::string& key) {
  std::string token = "\"" + key + "\"";
  auto key_pos = data.find(token);
  if (key_pos == std::string::npos) {
    return "";
  }
  auto colon_pos = data.find(':', key_pos + token.size());
  if (colon_pos == std::string::npos) {
    return "";
  }
  auto object_start = data.find('{', colon_pos + 1);
  if (object_start == std::string::npos) {
    return "";
  }

  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (size_t i = object_start; i < data.size(); ++i) {
    char c = data[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (c == '\\') {
      escaped = true;
      continue;
    }
    if (c == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (c == '{') {
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0) {
        return data.substr(object_start, i - object_start + 1);
      }
    }
  }
  return "";
}

std::string RobotHealthAggregator::EscapeJson(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

int RobotHealthAggregator::StatusRank(const std::string& status) {
  if (status == "ERROR" || status == "STALE" || status == "NO_DATA") {
    return 3;
  }
  if (status == "WARN") {
    return 2;
  }
  return 1;
}

std::string RobotHealthAggregator::RankStatus(int rank) {
  if (rank >= 3) {
    return "ERROR";
  }
  if (rank == 2) {
    return "WARN";
  }
  return "OK";
}

void RobotHealthAggregator::PublishHealth() {
  auto current_time = now();
  int overall_rank = 1;
  std::ostringstream modules;
  bool first = true;

  for (auto& item : states_) {
    auto& module = item.first;
    auto& state = item.second;
    std::string status = state.status;
    std::string code = state.code;
    std::string message = state.message;
    std::string metrics = ExtractJsonObject(state.raw, "metrics");
    double age = -1.0;

    if (state.received) {
      age = (current_time - state.last_seen).seconds();
      if (age > timeout_sec_) {
        status = "STALE";
        code = "HEALTH_TIMEOUT";
        message = "health timeout";
      }
    }

    overall_rank = std::max(overall_rank, StatusRank(status));

    if (!first) {
      modules << ",";
    }
    first = false;
    modules << "\"" << EscapeJson(module) << "\":{";
    modules << "\"topic\":\"" << EscapeJson(state.topic) << "\",";
    modules << "\"status\":\"" << EscapeJson(status) << "\",";
    modules << "\"code\":\"" << EscapeJson(code) << "\",";
    modules << "\"message\":\"" << EscapeJson(message) << "\",";
    modules << "\"age\":" << age << ",";
    modules << "\"timeout\":" << timeout_sec_;
    if (!metrics.empty() && status != "STALE") {
      modules << ",\"metrics\":" << metrics;
    }
    modules << "}";
  }

  std_msgs::msg::String msg;
  std::ostringstream payload;
  payload << "{";
  payload << "\"stamp\":" << current_time.seconds() << ",";
  payload << "\"overall\":\"" << RankStatus(overall_rank) << "\",";
  payload << "\"modules\":{" << modules.str() << "}";
  payload << "}";
  msg.data = payload.str();
  pub_health_->publish(msg);
}

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulator::RobotHealthAggregator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
