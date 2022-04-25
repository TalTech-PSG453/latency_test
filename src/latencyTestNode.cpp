#include <iostream>
#include <vector>
#include <chrono>
#include <ratio>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include <digital_twin_msgs/msg/supply_input.hpp>

#include "data_logger/data_logger.hpp"

using namespace DataLogger;
using namespace std::chrono_literals;

class LatencyTestNode : public rclcpp::Node
{
  public:
    std::unique_ptr<SubscriptionLogger> p_input_sub;

    LatencyTestNode() : Node("latency_test_node")
    {
      InputSubscriber_ = this->create_subscription<digital_twin_msgs::msg::SupplyInput>("/tb_tm/voltage", 10, 
                                                    std::bind(&LatencyTestNode::inputCallback, this, std::placeholders::_1));
      p_input_sub.reset(new SubscriptionLogger("/tb_tm/voltage"));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscription logger initialized");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LatencyTestNode initialized");
    }

  private:
    rclcpp::Subscription<digital_twin_msgs::msg::SupplyInput>::SharedPtr InputSubscriber_;

    void inputCallback(const digital_twin_msgs::msg::SupplyInput::SharedPtr msg)
    {
      if(msg->seq_id == p_input_sub->next_id) {
        auto now = rclcpp::Node::now();
        p_input_sub->recv_counter += 1;
        rclcpp::Duration diff = now - msg->stamp;
        auto num_of_us = diff.to_chrono<std::chrono::microseconds>();
        p_input_sub->time_diffs.push_back(num_of_us.count());
      } else {
        p_input_sub->lost_count += 1;
      }
      p_input_sub->next_id = msg->seq_id + 1;
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
