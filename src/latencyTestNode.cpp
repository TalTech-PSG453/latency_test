#include <iostream>
#include <vector>
#include <chrono>
#include <ratio>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include <digital_twin_msgs/msg/latency_test.hpp>
#include "std_msgs/msg/u_int64.hpp"

#include "data_logger/data_logger.hpp"

using namespace DataLogger;
using namespace std::chrono_literals;

class LatencyTestNode : public rclcpp::Node
{
  public:
    std::unique_ptr<SubscriptionLogger> p_input_sub;

    LatencyTestNode() : Node("latency_test_node")
    {
      PongPublisher_ = this->create_publisher<digital_twin_msgs::msg::LatencyTest>("/tb_tm/pong", 10);
      
      PingSubscriber_ = this->create_subscription<digital_twin_msgs::msg::LatencyTest>("/tb_tm/ping", 50, 
                                                    std::bind(&LatencyTestNode::pingCallback, this, std::placeholders::_1));
      
      LatencySubscriber_ = this->create_subscription<std_msgs::msg::UInt64>("/tb_tm/latency_results", 100, 
                                                    std::bind(&LatencyTestNode::latencyCallback, this, std::placeholders::_1));
      p_input_sub.reset(new SubscriptionLogger("/tb_tm/ping"));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscription logger initialized");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LatencyTestNode initialized");
    }

  private:
    rclcpp::Publisher<digital_twin_msgs::msg::LatencyTest>::SharedPtr PongPublisher_;
    rclcpp::Subscription<digital_twin_msgs::msg::LatencyTest>::SharedPtr PingSubscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr LatencySubscriber_;
    digital_twin_msgs::msg::LatencyTest msg_to_send;
    
    void pingCallback(const digital_twin_msgs::msg::LatencyTest::SharedPtr msg)
    {
      if(msg->seq_id == p_input_sub->next_id) {
        msg_to_send.seq_id = msg->seq_id;
        msg_to_send.stamp = msg->stamp;
        p_input_sub->recv_counter += 1;
        PongPublisher_->publish(msg_to_send);
      } else {
        p_input_sub->lost_count += 1;
      }
      p_input_sub->next_id = msg->seq_id + 1;
    }

    void latencyCallback(const std_msgs::msg::UInt64::SharedPtr msg){
      uint64_t latency_us = msg->data / 2;
      p_input_sub->time_diffs.push_back(latency_us);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto ptr = std::make_shared<LatencyTestNode>();
  rclcpp::spin(ptr);
  DataLogger::save_logged_data("latency_test_results.csv");
  rclcpp::shutdown();
  return 0;
}
