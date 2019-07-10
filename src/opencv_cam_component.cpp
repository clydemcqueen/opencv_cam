#include "opencv_cam/opencv_cam_component.hpp"

#include <iostream>
#include <fstream>

#include "cv_bridge/cv_bridge.h"

namespace opencv_cam
{

  bool get_camera_info(const std::string &camera_info_path, sensor_msgs::msg::CameraInfo &info)
  {
    // File format: 2 ints and 9 floats, separated by whitespace:
    // height width fx fy cx cy k1 k2 t1 t2 k3

    std::ifstream file;
    file.open(camera_info_path);
    if (!file) {
      return false;
    }

    uint32_t height, width;
    double fx, fy, cx, cy, k1, k2, t1, t2, k3;
    file >> height >> width;
    file >> fx >> fy;
    file >> cx >> cy;
    file >> k1 >> k2 >> t1 >> t2 >> k3;
    file.close();

    // See https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg

    info.header.frame_id = "camera_frame";
    info.height = height;
    info.width = width;
    info.distortion_model = "plumb_bob";

    info.d.push_back(k1);
    info.d.push_back(k2);
    info.d.push_back(t1);
    info.d.push_back(t2);
    info.d.push_back(k3);

    info.k[0] = fx;
    info.k[1] = 0;
    info.k[2] = cx;
    info.k[3] = 0;
    info.k[4] = fy;
    info.k[5] = cy;
    info.k[6] = 0;
    info.k[7] = 0;
    info.k[8] = 1;

    info.p[0] = fx;
    info.p[1] = 0;
    info.p[2] = cx;
    info.p[3] = 0;
    info.p[4] = 0;
    info.p[5] = fy;
    info.p[6] = cy;
    info.p[7] = 0;
    info.p[8] = 0;
    info.p[9] = 0;
    info.p[10] = 1;
    info.p[11] = 0;

    return true;
  }

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

    if (cxt_.str_api_) {
      camera_ = std::make_shared<cv::VideoCapture>(cxt_.filename_, cxt_.api_);
    } else {
      camera_ = std::make_shared<cv::VideoCapture>(cxt_.api_);
    }

    if (!camera_->isOpened()) {
      RCLCPP_ERROR(get_logger(), "cannot open camera");
      return;
    }

    if (!get_camera_info(cxt_.camera_info_path_, camera_info_msg_)) {
      RCLCPP_ERROR(get_logger(), "cannot get camera info");
    }

    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

    header_.frame_id = cxt_.camera_frame_;

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&OpencvCamNode::loop, this));

    RCLCPP_INFO(get_logger(), "publishing images and info");
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
      // Block until a frame is available
      camera_->read(frame);

      if (frame.rows == 0) {
        RCLCPP_INFO(get_logger(), "EOF, stop publishing");
        break;
      }

      // Skip some frames while debugging to slow down the pipeline
      static int skip_count = 0;
      if (++skip_count < cxt_.skip_frames_) {
        continue;
      }
      skip_count = 0;

      // Synchronize messages
      auto stamp = now();

      // Avoid copying image message if possible
      sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

      // Convert OpenCV Mat to ROS Image
      image_msg->header.stamp = stamp;
      image_msg->header.frame_id = "camera_frame";
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

#if 0
      // TODO better method to slow down the pipeline
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1s);
#endif
    }
  }

} // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCamNode)