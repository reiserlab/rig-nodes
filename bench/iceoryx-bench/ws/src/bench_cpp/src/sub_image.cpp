// C++ subscriber for sensor_msgs/Image. Latency = receive time - header.stamp (wall clock set by the publisher).
// Usage: sub_image <seconds>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stats.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  double secs = argc > 1 ? std::atof(argv[1]) : 10.0;
  auto node = std::make_shared<rclcpp::Node>("bench_sub_cpp");
  std::vector<int64_t> lat;
  size_t bytes = 0;
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    "bench", rclcpp::QoS(10).reliable(),
    [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
      int64_t t = now_ns();
      lat.push_back(t - (static_cast<int64_t>(m->header.stamp.sec) * 1000000000LL + m->header.stamp.nanosec));
      bytes = m->data.size();
    });
  auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(secs);
  // One executor for the whole run: rmw_iceoryx runs out of condition variables if a new one is made per spin.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    exec.spin_once(std::chrono::milliseconds(50));
  }
  (void)bytes;
  report(lat, "rclcpp");
  rclcpp::shutdown();
  return 0;
}
