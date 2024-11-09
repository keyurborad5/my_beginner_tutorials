#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

using namespace std::chrono_literals;

// parameter types
using PARAMETER_EVENT  = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HNADLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

class MyPublisher : public rclcpp::Node {
 public:
  MyPublisher() : Node("my_publisher"), count_(0) {
    
    
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is to set the rate of publisher!";
    this->declare_parameter("freq", 5.0, param_desc);
    auto freq = this->get_parameter("freq").as_double();

    // auto freq = param.get_parameter_value().get<std::float_t>(); // get new value, if any
    m_param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto paramCallbackPtr = std::bind (&MyPublisher::param_callback, this, std::placeholders::_1);
    m_paramHandle_ = m_param_subscriber_->add_parameter_callback ("freq", paramCallbackPtr);

    publisher_ =this->create_publisher<std_msgs::msg::String>("keyurs_topic", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>((1000/freq))), 
                                      std::bind(&MyPublisher::timer_callback, this));
    // Service to change string content dynamically
    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
        "change_string",
        std::bind(&MyPublisher::update_string, this, 
                  std::placeholders::_1,
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

  void param_callback (const rclcpp::Parameter & param) {
    RCLCPP_INFO (this->get_logger(),
                 "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                 param.get_name().c_str(),
                 param.get_type_name().c_str(),
                 param.as_double());

    // replace existing topic timer with a new one running at the new frequencey
    auto period = std::chrono::milliseconds ((int) (1000 / param.as_double()));
    auto topicCallbackPtr = std::bind (&MyPublisher::timer_callback, this);
    timer_ = this->create_wall_timer (period, topicCallbackPtr); // no memory leak here
  }
   
  PARAMETER_EVENT  m_param_subscriber_;
  PARAMETER_HNADLE m_paramHandle_;
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
