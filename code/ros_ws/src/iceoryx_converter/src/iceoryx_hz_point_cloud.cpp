#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "iceoryx_msg/msg/point_cloud640.hpp"

using namespace std::chrono_literals;

class IceoryxHzPointCloud : public rclcpp::Node
{
public:
  IceoryxHzPointCloud()
  : Node("iceoryx_hz_point_cloud")
  {
    auto qos = rclcpp::QoS(1);
    qos.best_effort();
    subscription_ = this->create_subscription<iceoryx_msg::msg::PointCloud640>(
      "points", qos, std::bind(
        &IceoryxHzPointCloud::image_callback, this,
        std::placeholders::_1));

    timer_ = this->create_wall_timer(
      1s, std::bind(&IceoryxHzPointCloud::print_hz, this));
  }

private:
  void print_hz() {
    times_mutex_.lock();
    if (times_.size() == 0) {
      std::cout << "No measurements yet." << std::endl;
      times_mutex_.unlock();
      return;
    }
    double sum;
    for (auto duration : times_) {
      sum += duration / 1e+6;
    }
    auto mean = (double) sum / (double) times_.size();
    times_mutex_.unlock();
    auto hz = 1.0 / mean;
    std::cout << "Current Hz: " << hz << std::endl;
  }

  void image_callback(iceoryx_msg::msg::PointCloud640::SharedPtr point_cloud)
  {
    times_mutex_.lock();
    auto time = std::chrono::high_resolution_clock::now();
    if (first_) {
      last_ = time;
      first_ = false;
    } else {
      int duration = std::chrono::duration_cast<std::chrono::microseconds>(time - last_).count();
      times_.push_back(duration);
      last_ = time;
    }
    if (times_.size() > 10) {
      times_.erase(times_.begin());
    }
    times_mutex_.unlock();
  }

  bool first_ = true;
  std::chrono::high_resolution_clock::time_point last_;
  std::vector<long long int> times_;
  rclcpp::Subscription<iceoryx_msg::msg::PointCloud640>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex times_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceoryxHzPointCloud>());
  rclcpp::shutdown();
  return 0;
}
