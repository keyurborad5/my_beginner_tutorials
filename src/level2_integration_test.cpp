/**
 * @file level2_integration_test.cpp
 * @author Keyur Borad (keyurborad5@gmail.com)
 * @brief This cpp file is a level2 integration file to verify the unit test of publishing topic "keyurs_topic"
 * @version 0.1
 * @date 2024-11-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


using namespace std::chrono_literals;
using std_msgs::msg::String;

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger("");  // create an initial Logger
/**
 * @brief Class MyTestsFixure to make a test fixtures
 * 
 */
class MyTestsFixture {
 public:
 /**
  * @brief Construct a new My Tests Fixture object
  * 
  */
  MyTestsFixture() {
    /**
     * 1.) Create the node that performs the test. (aka Integration test node):
     */
    testerNode = rclcpp::Node::make_shared("Level2IntegrationTest");
    Logger =
        testerNode
            ->get_logger();  // make sure message will appear in rqt_console

    /**
     * 2.) Declare a parameter for the duration of the test:
     */
    testerNode->declare_parameter<double>("test_duration");

    /**
     * 3.) Get the test duration value:
     */
    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture() {}

 protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

////////////////////////////////////////////////
// Test Case 1
////////////////////////////////////////////////

/* In this test case, the node under test (aka Auxiliary test node)
   is a topic talker, which got launched by the launcher.

   We will ceate a topic listener as part of the node performing the
   test (aka Integration test node).  And the test simply checks if
   the topic is recieved within the duration of the test. */

TEST_CASE_METHOD(MyTestsFixture, "test topic talker", "[topic]") {
  /**
   * 4.) Now, subscribe to a specific topic we're looking for:
   */
  bool got_topic = false;

  // Define a callback that captures the additional parameter
  struct ListenerCallback {
    explicit ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic) {}
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM(Logger, "I heard:" << msg.data.c_str());
      gotTopic_ = true;
    }
    bool &gotTopic_;
  };

  auto subscriber = testerNode->create_subscription<String>(
      "keyurs_topic", 10, ListenerCallback(got_topic));

  /**
   * 5.) Finally do the actual test:
   */
  rclcpp::Rate rate(5.0);  // 5hz checks
  auto start_time = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - start_time;
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);
  RCLCPP_INFO_STREAM(Logger, "duration = " << duration.seconds()
                                           << " timeout=" << timeout.seconds());
  while (!got_topic && (duration < timeout)) {
    rclcpp::spin_some(testerNode);
    rate.sleep();
    duration = (rclcpp::Clock().now() - start_time);
  }

  RCLCPP_INFO_STREAM(Logger, "duration = " << duration.seconds()
                                           << " got_topic=" << got_topic);
  CHECK(got_topic);  // Test assertions - check that the topic was received
}
