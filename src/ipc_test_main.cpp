#include "opencv_cam/opencv_cam_node.hpp"
#include "opencv_cam/subscriber_node.hpp"

// Manually compose OpencvCamNode and ImageSubscriberNode with use_intra_process_comms=true

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create and add camera node
  rclcpp::NodeOptions options{};
  options.use_intra_process_comms(true);
  auto camera_node = std::make_shared<opencv_cam::OpencvCamNode>(options);
  executor.add_node(camera_node);

  // Create and add subscriber node
  auto subscriber_node = std::make_shared<opencv_cam::ImageSubscriberNode>(options);
  executor.add_node(subscriber_node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
