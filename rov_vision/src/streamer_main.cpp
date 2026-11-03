#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rov_vision/streamer.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rov_vision::Streamer>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("streamer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
