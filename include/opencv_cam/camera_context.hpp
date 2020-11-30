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
  CXT_MACRO_MEMBER(camera_info_path, std::string, "info.ini")     /* Camera info path */ \
  CXT_MACRO_MEMBER(camera_frame_id, std::string, "camera_frame")  /* Camera frame id */ \
/* End of list */


  struct CameraContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    CXT_MACRO_DEFINE_MEMBERS(OPENCV_CAM_ALL_PARAMS)
  };

} // namespace opencv_cam

#endif // OPENCV_CAM_CONTEXT_HPP
