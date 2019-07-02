#ifndef OPENCV_CAM_CONTEXT_HPP
#define OPENCV_CAM_CONTEXT_HPP

#include <math.h>
#include <string>

#include "ros2_shared/context_macros.hpp"

namespace opencv_cam
{

#define OPENCV_CAM_ALL_PARAMS \
  CXT_MACRO_MEMBER(str_api, bool, false)                          /* True: use VideoCapture(filename, api) API */ \
  CXT_MACRO_MEMBER(filename, std::string, "")                     /* Filename, ignored if str_api is false */ \
  CXT_MACRO_MEMBER(api, int, 0)                                   /* Index, see cv::VideoCaptureAPIs */ \
  CXT_MACRO_MEMBER(skip_frames, int, 0)                           /* Target fps = 30 / (skip + 1) */ \
  CXT_MACRO_MEMBER(camera_info_path, std::string, "")             /* Camera info path */ \
  CXT_MACRO_MEMBER(camera_frame, std::string, "camera_frame")     /* Camera frame */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct CameraContext
  {
    OPENCV_CAM_ALL_PARAMS
  };

} // namespace opencv_cam

#endif // OPENCV_CAM_CONTEXT_HPP
