#include "opencv_cam/subscriber_node.hpp"

namespace opencv_cam
{

  ImageSubscriberNode::ImageSubscriberNode(const rclcpp::NodeOptions &options) :
    Node("image_subscriber", options)
  {
    RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 1,
      [this](sensor_msgs::msg::Image::UniquePtr msg)
      {
        static bool receiving = false;

        if (!receiving) {
          receiving = true;
          RCLCPP_INFO(get_logger(), "receiving messages");
        }

#undef SLOW_DOWN
#ifdef SLOW_DOWN
        // Overflow the pub and sub buffer
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
#endif

#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
        static int count = 0;
        RCLCPP_INFO(get_logger(), "%d, %p", count++, reinterpret_cast<std::uintptr_t>(msg.get()));
#else
        (void) this;
        (void) msg;
#endif
      });

    RCLCPP_INFO(get_logger(), "ready");
  }

} // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::ImageSubscriberNode)