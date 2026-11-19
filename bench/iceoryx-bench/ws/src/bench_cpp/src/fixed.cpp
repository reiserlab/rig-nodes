// Fixed-size message publisher/subscriber using the loaned-message API (the zero-copy path).
// Usage: fixed pub|sub 64k|2m|8m <rate_hz> <seconds>
#include <cstdlib>
#include <cstring>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "bench_msgs/msg/fixed64_k.hpp"
#include "bench_msgs/msg/fixed2_m.hpp"
#include "bench_msgs/msg/fixed8_m.hpp"
#include "stats.hpp"

template<typename MsgT>
int run(const std::string & role, double rate, double secs) {
  auto node = std::make_shared<rclcpp::Node>("bench_fixed_" + role);
  auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(secs);
  if (role == "pub") {
    auto pub = node->create_publisher<MsgT>("bench_fixed", rclcpp::QoS(10).reliable());
    std::printf("{\"can_loan\":%s}\n", pub->can_loan_messages() ? "true" : "false");
    std::this_thread::sleep_for(std::chrono::seconds(2));  // let the subscriber match
    auto period = std::chrono::duration<double>(1.0 / rate);
    auto next = std::chrono::steady_clock::now();
    size_t n = 0;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      auto loaned = pub->borrow_loaned_message();
      loaned.get().data[0] = static_cast<uint8_t>(n);  // touch the payload, as a real producer would write it
      loaned.get().stamp_ns = now_ns();
      pub->publish(std::move(loaned));
      ++n;
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }
    std::printf("{\"sent\":%zu}\n", n);
  } else {
    std::vector<int64_t> lat;
    auto sub = node->create_subscription<MsgT>(
      "bench_fixed", rclcpp::QoS(10).reliable(),
      [&](std::shared_ptr<const MsgT> m) {
        int64_t t = now_ns();
        lat.push_back(t - m->stamp_ns);
      });
    // One executor for the whole run: rmw_iceoryx runs out of condition variables if a new one is made per spin.
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      exec.spin_once(std::chrono::milliseconds(50));
    }
    report(lat, "rclcpp-fixed");
  }
  return 0;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  if (argc < 5) { std::fprintf(stderr, "usage: fixed pub|sub 64k|2m|8m rate secs\n"); return 2; }
  std::string role = argv[1], size = argv[2];
  double rate = std::atof(argv[3]), secs = std::atof(argv[4]);
  int rc;
  if (size == "64k") rc = run<bench_msgs::msg::Fixed64K>(role, rate, secs);
  else if (size == "2m") rc = run<bench_msgs::msg::Fixed2M>(role, rate, secs);
  else rc = run<bench_msgs::msg::Fixed8M>(role, rate, secs);
  rclcpp::shutdown();
  return rc;
}
