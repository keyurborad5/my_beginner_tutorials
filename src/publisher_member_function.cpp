#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MyPublisher : public rclcpp::Node {
 public:
  MyPublisher() : Node("my_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("keyurs_topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MyPublisher::timer_callback, this));
    
    // Service to change string content dynamically
    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
        "change_string",
        std::bind(&MyPublisher::update_string, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data =  service_msg+" Says Hi! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  // }
  }

   void update_string(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response) {
    service_msg = request->input;
    response->output = request->input;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Incoming request, Change string to: ["
                           << request->input.c_str() << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Sending back response: ["
                                               << response->output.c_str()
                                               << "]");
  } 
  rclcpp::TimerBase::SharedPtr timer_;
  std::string service_msg = "Keyur";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}
