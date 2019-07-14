#include "opencv_cam/opencv_cam_node.hpp"

#include <iostream>
#include <fstream>

#include "camera_calibration_parsers/parse.h"

namespace opencv_cam
{

  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("unsupported encoding type");
    }
  }

  OpencvCamNode::OpencvCamNode(const rclcpp::NodeOptions &options) :
    Node("opencv_cam", options)
  {
    // Get parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), OPENCV_CAM_ALL_PARAMS, validate_parameters)

    RCLCPP_INFO(get_logger(), "OpenCV version %d", CV_VERSION_MAJOR);

    std::string capture_name = cxt_.file_ ? "file" : "device";

    // Open file or device
    if (cxt_.file_) {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.filename_, cxt_.index_);
    } else {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.index_);
    }

    if (!capture_->isOpened()) {
      RCLCPP_ERROR(get_logger(), "cannot open %s", capture_name.c_str());
      return;
    }

    double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
    double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
    RCLCPP_INFO(get_logger(), "%s open, width = %g, height = %g", capture_name.c_str(), width, height);

    if (cxt_.file_) {
      if (cxt_.fps_ > 0) {
        // Publish at the specified rate
        fps_ = cxt_.fps_;
      } else {
        // Publish at the recorded rate
        fps_ = capture_->get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(get_logger(), "publish at %d fps", fps_);
      }
      next_stamp_ = now();
    } else {
      // Publish at the device rate
      fps_ = 0;
    }

    std::string camera_name;
    if (camera_calibration_parsers::readCalibration(cxt_.camera_info_path_, camera_name, camera_info_msg_)) {
      RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "cannot get camera info");
    }

    camera_info_msg_.header.frame_id = cxt_.camera_frame_id_;

    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&OpencvCamNode::loop, this));

    RCLCPP_INFO(get_logger(), "start publishing");
  }

  OpencvCamNode::~OpencvCamNode()
  {
    // Stop loop
    canceled_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void OpencvCamNode::validate_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    OPENCV_CAM_ALL_PARAMS
  }

  void OpencvCamNode::loop()
  {
    cv::Mat frame;

    while (rclcpp::ok() && !canceled_.load()) {
      // Read a frame, if this is a device block until a frame is available
      if (!capture_->read(frame)) {
        RCLCPP_INFO(get_logger(), "EOF, stop publishing");
        break;
      }

      auto stamp = now();
      camera_info_msg_.header.stamp = stamp;

      // Avoid copying image message if possible
      sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

      // Convert OpenCV Mat to ROS Image
      image_msg->header.stamp = stamp;
      image_msg->header.frame_id = cxt_.camera_frame_id_;
      image_msg->height = frame.rows;
      image_msg->width = frame.cols;
      image_msg->encoding = mat_type2encoding(frame.type());
      image_msg->is_bigendian = false;
      image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      image_msg->data.assign(frame.datastart, frame.dataend);

#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
      static int count = 0;
      RCLCPP_INFO(get_logger(), "%d, %p", count++, reinterpret_cast<std::uintptr_t>(image_msg.get()));
#endif

      // Publish
      image_pub_->publish(std::move(image_msg));
      camera_info_pub_->publish(camera_info_msg_);

      // Sleep if required
      if (fps_ > 0) {
        next_stamp_ = next_stamp_ + rclcpp::Duration{1000000000L / fps_};
        auto wait = next_stamp_ - stamp;
        if (wait.nanoseconds() > 0) {
          std::this_thread::sleep_for(static_cast<std::chrono::nanoseconds>(wait.nanoseconds()));
        }
      }
    }
  }

} // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCamNode)