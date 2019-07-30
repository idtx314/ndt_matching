#include <cstdio>
#include <memory>
#include <string>
#include "ndt_lib.cpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

void print_usage()
{
  printf("Usage for listener app:\n");
  printf("listener [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to subscribe. Defaults to chatter.\n");
}

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string & topic_name, const std::string & topic_name2 = "map")
  : Node("listener") //ID: Creation of the node object
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->header.frame_id.c_str());
        //TODO:
        // Call NdtLib function and pass in the input message.
        // Create NdtLib object
        ndt_matching::NdtLib ndt_object = ndt_matching::NdtLib();


        // Test Output
        int num = 4;
        RCLCPP_INFO(this->get_logger(), "msg2: [%d]", num);

        // return a pose message


        // Form posestamped message for publishing
        auto msg_out = std::make_shared<geometry_msgs::msg::PoseStamped>();

        //TODONE:
        // Publish the pose as a PoseStamped message.
        pub_->publish(msg_out);

      };

    auto callback2 =
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->header.frame_id.c_str());
        //TODO: here you get your map point cloud (one time only)

      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, callback);
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name2, callback2);
    // TODONE: create a pose publisher, see for reference
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("filtered_points", rmw_qos_profile_default);
    // rclcpp::Rate loop_rate(10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("points_raw");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic = std::string(cli_option);
  }

  // Create a node.
  auto node = std::make_shared<Listener>(topic);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
