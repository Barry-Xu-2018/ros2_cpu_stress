#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

class StressSubscriber : public rclcpp::Node
{
public:
  StressSubscriber(const rclcpp::NodeOptions &node_options)
  : Node("stress_subscriber", node_options)
  {
    auto best_effort = this->declare_parameter("best_effort", false);

    rclcpp::QoS qos(10);
    if (best_effort)
      qos.best_effort();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "topic", qos, std::bind(&StressSubscriber::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "best_effort : %s", best_effort ? "true" : "false");

    first_ = true;
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    auto current = this->now();

    if (first_) {
      acc_count_ = 0;
      acc_start_ = this->now();
      acc_size_ = 0;
      acc_latency_ = 0;
      last_frame_id_ = msg->step;
    }

    // check bandwidth and latency
    auto elapse = (current - acc_start_).seconds();
    if (elapse > 5.0) {
      RCLCPP_INFO(this->get_logger(), "bandwidth : %.1f MB/s, latency : %.2f ms",
                  (acc_size_ / elapse) / 1024 / 1024, (acc_latency_ * 1000.0) / acc_count_);
      acc_count_ = 0;
      acc_start_ = current;
      acc_size_ = 0;
      acc_latency_ = 0;
    }
    acc_count_++;
    acc_size_ += msg->data.size();
    acc_latency_ += (current - rclcpp::Time(msg->header.stamp)).seconds();

    // check frame drop
    auto frame_id = msg->step;
    if (!first_ && (frame_id - last_frame_id_) != 1) {
      RCLCPP_WARN(this->get_logger(), "framedrop detected : last %d, current %d",
                  last_frame_id_, frame_id);
    }
    last_frame_id_ = frame_id;

    // clear first flag
    first_ = false;
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  bool first_;
  unsigned int acc_count_;
  rclcpp::Time acc_start_;
  unsigned int acc_size_;
  double acc_latency_;
  unsigned int last_frame_id_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(StressSubscriber);
