#include "opencv_cam/opencv_cam_node.hpp"

// Launch OpencvCamNode with use_intra_process_comms=true

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
  auto node = std::make_shared<opencv_cam::OpencvCamNode>(options);
  executor.add_node(node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
