#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

class StressPublisher : public rclcpp::Node
{
public:
  StressPublisher(const rclcpp::NodeOptions &node_options)
  : Node("stress_publisher", node_options)
  {
    auto size_mb = this->declare_parameter("size_mb", 100);
    auto hz = this->declare_parameter("hz", 10);
    auto best_effort = this->declare_parameter("best_effort", false);

    rclcpp::QoS qos(10);
    if (best_effort)
      qos.best_effort();

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", qos);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / hz),
                                     std::bind(&StressPublisher::timer_callback, this));

    message_.step = 0;
    message_.data.resize(size_mb * 1024 * 1024);

    RCLCPP_INFO(this->get_logger(), "size_mb : %ld, hz : %ld, best_effort : %s",
                size_mb, hz, best_effort ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Publishing %ld MB/s", size_mb * hz);
  }

private:
  void timer_callback()
  {
    message_.header.stamp = this->now();
    message_.step++;
    publisher_->publish(message_);
  }
  sensor_msgs::msg::Image message_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(StressPublisher);
