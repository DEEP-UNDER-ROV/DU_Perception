#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rov_vision/depth_estimation.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rov_vision::DepthEstimation>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("depth_estimation"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
