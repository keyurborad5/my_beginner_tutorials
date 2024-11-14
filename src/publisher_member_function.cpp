/**
 * @file publisher_member_function.cpp
 * @author Keyur Borad (keyurborad5@gmail.com)
 * @brief This is a publisher cpp file to publish custom message at a custom
 * rate
 * @version 0.1
 * @date 2024-11-08
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

// parameter types
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HNADLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/**
 * @brief MyPublisher class to publish the custom message
 *
 */
class MyPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new My Publisher object
   *
   */
  MyPublisher() : Node("my_publisher"), count_(0) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    /////////////////////////// Parameter ///////////////////////////

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is to set the rate of publisher!";
    this->declare_parameter("freq", 5.0, param_desc);
    auto freq = this->get_parameter("freq").as_double();

    ////////////// Parameter Event Handler Subscriber ///////////////
    m_param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    m_paramHandle_ = m_param_subscriber_->add_parameter_callback(
        "freq",
        std::bind(&MyPublisher::param_callback, this, std::placeholders::_1));

    /////////////////////////// Publisher ///////////////////////////

    publisher_ =
        this->create_publisher<std_msgs::msg::String>("keyurs_topic", 10);

    /////////////////////////// Timer ///////////////////////////

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>((1000 / freq))),
        std::bind(&MyPublisher::timer_callback, this));

    /////////////////////////// Service ///////////////////////////
    // Service to change string content dynamically
    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
        "change_string",
        std::bind(&MyPublisher::update_string, this, std::placeholders::_1,
                  std::placeholders::_2));
    /////////////////////////// Broadcaster ///////////////////////////

    tf_static_broadcaster_= std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->make_transforms();
    
  }

 private:
  /////////////////////////// Methods ///////////////////////////

  /**
   * @brief timer_callback memeber method a callback function from the publisher
   * which calls this callback method in user defined rate and publishes
   *
   */
  void timer_callback() {
    // A conditional statement to check if the rate is above 100 msgs/sec
    if (this->get_parameter("freq").as_double() > 100.0) {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "FATAL publisher rate: "
              << this->get_parameter("freq").as_double()
              << " per sec. Cannot handle so high publishing rate");
      rclcpp::shutdown();
    } else {
      auto message = std_msgs::msg::String();
      message.data = service_msg + " Says Hi! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // Print publisher rate after every 10 iterations to debug
      if (count_ % 20 == 0) {
        RCLCPP_DEBUG_STREAM(
            this->get_logger(),
            "publishing rate: " << this->get_parameter("freq").as_double());
      }
      publisher_->publish(message);
    }
  }
  /**
   * @brief update_string callback method fro services, whenever the services
   * are called this method is called to update the service_msg(here)
   *
   * @param request : input(string)
   * @param response : output(string)
   */
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

  /**
   * @brief param_callback method to update the param value. As param values do
   * not require seperate callback function to dynamically change the param
   * value but here as the param value is used to set the rate of publisher and
   * the rate is only called once whenever the constructor is called. Hence we
   * need to set a seperate callback function which re-initialises the timer
   * module
   *
   * @param param
   */
  void param_callback(const rclcpp::Parameter& param) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Received an update to parameter"
                                               << param.get_name().c_str()
                                               << " of type "
                                               << param.get_type_name().c_str()
                                               << " : " << param.as_double());

    // replace existing topic timer with a new one running at the new frequencey
    auto period = std::chrono::milliseconds((int)(1000 / param.as_double()));
    auto topicCallbackPtr = std::bind(&MyPublisher::timer_callback, this);
    timer_ = this->create_wall_timer(period,
                                     topicCallbackPtr);  // no memory leak here
  }

  void make_transforms(){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 5;
    t.transform.translation.y = -20;
    t.transform.translation.z = 10;
    tf2::Quaternion q;
    q.setRPY(1.57,0.349,-2.0944);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  /////////////////////////// Attributes ///////////////////////////
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  PARAMETER_EVENT m_param_subscriber_;
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
