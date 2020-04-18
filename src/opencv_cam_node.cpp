#include "opencv_cam/opencv_cam_node.hpp"

#include <iostream>
#include <fstream>

#include "camera_calibration_parsers/parse.h"

namespace opencv_cam
{


#define TEST(s) if (vcp_name == #s) {return cv::s;}

  static int select_video_capture_property(const std::string &vcp_name)
  {
    TEST(CAP_PROP_POS_MSEC);
    TEST(CAP_PROP_POS_FRAMES);
    TEST(CAP_PROP_POS_AVI_RATIO);
    TEST(CAP_PROP_FRAME_WIDTH);
    TEST(CAP_PROP_FRAME_HEIGHT);
    TEST(CAP_PROP_FPS);
    TEST(CAP_PROP_FOURCC);
    TEST(CAP_PROP_FRAME_COUNT);
    TEST(CAP_PROP_FORMAT);
    TEST(CAP_PROP_MODE);
    TEST(CAP_PROP_BRIGHTNESS);
    TEST(CAP_PROP_CONTRAST);
    TEST(CAP_PROP_SATURATION);
    TEST(CAP_PROP_HUE);
    TEST(CAP_PROP_GAIN);
    TEST(CAP_PROP_EXPOSURE);
    TEST(CAP_PROP_CONVERT_RGB);
    TEST(CAP_PROP_WHITE_BALANCE_BLUE_U);
    TEST(CAP_PROP_RECTIFICATION);
    TEST(CAP_PROP_MONOCHROME);
    TEST(CAP_PROP_SHARPNESS);
    TEST(CAP_PROP_AUTO_EXPOSURE);
    TEST(CAP_PROP_GAMMA);
    TEST(CAP_PROP_TEMPERATURE);
    TEST(CAP_PROP_TRIGGER);
    TEST(CAP_PROP_TRIGGER_DELAY);
    TEST(CAP_PROP_WHITE_BALANCE_RED_V);
    TEST(CAP_PROP_ZOOM);
    TEST(CAP_PROP_FOCUS);
    TEST(CAP_PROP_GUID);
    TEST(CAP_PROP_ISO_SPEED);
    TEST(CAP_PROP_BACKLIGHT);
    TEST(CAP_PROP_PAN);
    TEST(CAP_PROP_TILT);
    TEST(CAP_PROP_ROLL);
    TEST(CAP_PROP_IRIS);
    TEST(CAP_PROP_SETTINGS);
    TEST(CAP_PROP_BUFFERSIZE);
    TEST(CAP_PROP_AUTOFOCUS);
    TEST(CAP_PROP_SAR_NUM);
    TEST(CAP_PROP_SAR_DEN);
    TEST(CAP_PROP_BACKEND);
    TEST(CAP_PROP_CHANNEL);
    TEST(CAP_PROP_AUTO_WB);
    TEST(CAP_PROP_WB_TEMPERATURE);
    return -1;
  }

  static void set_video_capture_property(const std::string &vcp_name,
                                         const double &vcp_value,
                                         rclcpp::Logger &logger,
                                         std::shared_ptr<cv::VideoCapture> &capture)
  {
    if (!vcp_name.empty()) {
      auto vcp_id = select_video_capture_property(vcp_name);

      if (vcp_id < 0) {
        RCLCPP_ERROR(logger, "Video Capture Property unknown: %s", vcp_name.c_str());
        return;
      }

      RCLCPP_INFO(logger, "Video Capture Property '%s' set to %f", vcp_name.c_str(), vcp_value);
      capture->set(vcp_id, vcp_value);
    }
  }

  static void set_video_capture_properties(const CameraContext &cxt,
                                           rclcpp::Logger logger,
                                           std::shared_ptr<cv::VideoCapture> &capture)
  {
    set_video_capture_property(cxt.vcp_property0_, cxt.vcp_value0_, logger, capture);
    set_video_capture_property(cxt.vcp_property1_, cxt.vcp_value1_, logger, capture);
    set_video_capture_property(cxt.vcp_property2_, cxt.vcp_value2_, logger, capture);
    set_video_capture_property(cxt.vcp_property3_, cxt.vcp_value3_, logger, capture);
    set_video_capture_property(cxt.vcp_property4_, cxt.vcp_value4_, logger, capture);
    set_video_capture_property(cxt.vcp_property5_, cxt.vcp_value5_, logger, capture);
    set_video_capture_property(cxt.vcp_property6_, cxt.vcp_value6_, logger, capture);
    set_video_capture_property(cxt.vcp_property7_, cxt.vcp_value7_, logger, capture);
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
    // Initialize parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Register for parameter changed message
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Display parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
    CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "OpenCV Parameters", OPENCV_CAM_ALL_PARAMS)

    // Check that all command line parameters are registered
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), OPENCV_CAM_ALL_PARAMS)

    RCLCPP_INFO(get_logger(), "OpenCV version %d", CV_VERSION_MAJOR);

    std::string capture_name{};

    // Open file or device
    if (cxt_.file_) {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.filename_, cxt_.index_);
      capture_name = std::string("Video file:'")
        .append(cxt_.filename_)
        .append("' on index:")
        .append(std::to_string(cxt_.index_));
    } else {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.index_);
      capture_name = std::string("Video device on index:")
        .append(std::to_string(cxt_.index_));
//
//      capture_->set(cv::CAP_PROP_FRAME_WIDTH, 2560);
//      capture_->set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    }

    if (!capture_->isOpened()) {
      RCLCPP_ERROR(get_logger(), "cannot open %s", capture_name.c_str());
      return;
    }

    set_video_capture_properties(cxt_, get_logger(), capture_);

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
  {}

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
      image_msg->width = cxt_.half_image_ ? frame.cols / 2 : frame.cols;
      image_msg->encoding = mat_type2encoding(frame.type());
      image_msg->is_bigendian = false;
      image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(cxt_.half_image_ ? frame.step / 2 : frame.step);

      // Copy the data from the mat to the message
      if (cxt_.half_image_ == 0) {
        image_msg->data.assign(frame.datastart, frame.dataend);
      } else {
        int bytes_per_row = image_msg->width * frame.channels();
        for (int row = 0; row < frame.rows; row += 1) {
          uint8_t *p = frame.ptr<uint8_t>(row) + (cxt_.half_image_ == 1 ? 0 : bytes_per_row);
          image_msg->data.insert(image_msg->data.end(), p, p + bytes_per_row);
        }
      }

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