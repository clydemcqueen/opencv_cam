#ifndef OPENCV_CAM_CONTEXT_HPP
#define OPENCV_CAM_CONTEXT_HPP

#include <cmath>
#include <string>

#include "ros2_shared/context_macros.hpp"

namespace opencv_cam
{

#define OPENCV_CAM_ALL_PARAMS \
  CXT_MACRO_MEMBER(file, bool, false)                             /* Read from file vs. read from device */ \
  CXT_MACRO_MEMBER(fps, int, 0)                                   /* Desired frames per second */ \
  \
  CXT_MACRO_MEMBER(filename, std::string, "")                     /* Filename */ \
  \
  CXT_MACRO_MEMBER(index, int, 0)                                 /* Device index, see cv::VideoCaptureAPIs */ \
  CXT_MACRO_MEMBER(width, int, 0)                                 /* Device width */ \
  CXT_MACRO_MEMBER(height, int, 0)                                /* Device height */ \
  \
  CXT_MACRO_MEMBER(camera_info_path, std::string, "")             /* Camera info path */ \
  CXT_MACRO_MEMBER(camera_frame_id, std::string, "camera_frame")  /* Camera frame id for the header in the camera_info message */ \
  \
  CXT_MACRO_MEMBER(vcp_property0, std::string, "")               /* Video Capture Property - property0 */ \
  CXT_MACRO_MEMBER(vcp_property1, std::string, "")               /* Video Capture Property - property1 */ \
  CXT_MACRO_MEMBER(vcp_property2, std::string, "")               /* Video Capture Property - property2 */ \
  CXT_MACRO_MEMBER(vcp_property3, std::string, "")               /* Video Capture Property - property3 */ \
  CXT_MACRO_MEMBER(vcp_property4, std::string, "")               /* Video Capture Property - property4 */ \
  CXT_MACRO_MEMBER(vcp_property5, std::string, "")               /* Video Capture Property - property5 */ \
  CXT_MACRO_MEMBER(vcp_property6, std::string, "")               /* Video Capture Property - property6 */ \
  CXT_MACRO_MEMBER(vcp_property7, std::string, "")               /* Video Capture Property - property7 */ \
  CXT_MACRO_MEMBER(vcp_value0, double, 0.)                       /* Video Capture Property - value0 */ \
  CXT_MACRO_MEMBER(vcp_value1, double, 0.)                       /* Video Capture Property - value1 */ \
  CXT_MACRO_MEMBER(vcp_value2, double, 0.)                       /* Video Capture Property - value2 */ \
  CXT_MACRO_MEMBER(vcp_value3, double, 0.)                       /* Video Capture Property - value3 */ \
  CXT_MACRO_MEMBER(vcp_value4, double, 0.)                       /* Video Capture Property - value4 */ \
  CXT_MACRO_MEMBER(vcp_value5, double, 0.)                       /* Video Capture Property - value5 */ \
  CXT_MACRO_MEMBER(vcp_value6, double, 0.)                       /* Video Capture Property - value6 */ \
  CXT_MACRO_MEMBER(vcp_value7, double, 0.)                       /* Video Capture Property - value7 */ \
  \
  CXT_MACRO_MEMBER(half_image, int, 0)                           /* for stereo image take: 0: whole, 1: left half, 2:right half */ \
/* End of list */


  struct CameraContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    CXT_MACRO_DEFINE_MEMBERS(OPENCV_CAM_ALL_PARAMS)
  };

} // namespace opencv_cam

#endif // OPENCV_CAM_CONTEXT_HPP
