#include "opencv_cam/opencv_cam_component.hpp"

// Node that consumes messages.
class ImageSubscriberNode : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

public:

  ImageSubscriberNode(const rclcpp::NodeOptions &options) :
    Node("ipc_test", options)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, [](sensor_msgs::msg::Image::UniquePtr msg)
      {
        static int count = 0;
        static bool eof = false;

        if (count == 0) {
          std::cout << "receiving messages" << std::endl;
        }

        count++;

        if (!eof && msg->width == 0) {
          std::cout << "count was " << count << std::endl;
          eof = true;
        }

#if 1
        std::cout << "Recv address: " << reinterpret_cast<std::uintptr_t>(msg.get()) << std::endl;
#endif
      });
  }
};


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
  auto subscriber_node = std::make_shared<ImageSubscriberNode>(options);
  executor.add_node(subscriber_node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
