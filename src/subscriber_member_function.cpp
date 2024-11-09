/**
 * @file subscriber_member_function.cpp
 * @author keyur borad (keyurbora5@gmail.com)
 * @brief This is a subcriber cpp file which subscribers and loggs the custom message
 * @version 0.1
 * @date 2024-11-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
/**
 * @brief MySubscriber Class to define a subscriber that logs the message published on a given topic
 * 
 */
class MySubscriber : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new My Subscriber object
  * 
  */
  MySubscriber() : Node("my_subscriber") {
    //Creating a subsciber
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "keyurs_topic", 10, std::bind(&MySubscriber::topic_callback, this, _1));
    //Flag for logging the error message if subscriber node is running and publisher is not
    msg_received_ = false;
    // create wall timer to check regularly for messages
    timer_ = this->create_wall_timer(
                  std::chrono::seconds(1),
                  std::bind(&MySubscriber::check_for_msgs, this));
  }

 private:
  /////////////////////////// Methods ///////////////////////////

 /**
  * @brief topic_callback member method to log the messages
  * 
  * @param msg 
  */
  void topic_callback(const std_msgs::msg::String& msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    msg_received_ = true;
  }

  /**
   * @brief check_for_msgs member method to check is any message is received from the given topic
   * 
   */
  void check_for_msgs() {
    if (!msg_received_) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Topic 'keyurs_topic' not published yet..");
    }
  }

  /////////////////////////// Attributes ///////////////////////////
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool msg_received_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}
