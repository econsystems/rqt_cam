#ifndef SERVICES_H
#define SERVICES_H
#include "ecam_v4l2/camera.h"
#include "ecam_v4l2/device.h"
#include "ecam_v4l2/query_control.h"
#include "ecam_v4l2/set_control.h"
#include "ecam_v4l2/set_format.h"
#include "ecam_v4l2/enum_format.h"
#include "ecam_v4l2/V4l2.h"
#include <sys/mman.h>
#include <string>

namespace ecam_v4l2
{
 class Services
 {
   private:
     Devices &dev; // container to hold obj of class Devices
     std::vector<Camera>& cam; // container to hold obj of class Camera
     struct v4l2_queryctrl queryctrl;
     std::vector<int> stream_list;  // list of cameras which are streaming


	 public:
     Services(Devices &Devices_obj ,std::vector<Camera>& Camera_obj);
     bool onCameraChange(ecam_v4l2::camera::Request &req,ecam_v4l2::camera::Response &res);
     bool isControlAvailable(ecam_v4l2::query_control::Request &req,ecam_v4l2::query_control::Response &res);
     bool onSetControl(ecam_v4l2::set_control::Request &req,ecam_v4l2::set_control::Response &res);
     bool onEnumFormat(ecam_v4l2::enum_format::Request &req,ecam_v4l2::enum_format::Response &res);
     bool onFormatChange(ecam_v4l2::set_format::Request &req,ecam_v4l2::set_format::Response &res);
     bool get_current_device(std::string name,int *index);
     bool isNotStreaming(int index);
     bool check_valid_control(std::string control_name);
     void add_camera_streaming(int index);
     void remove_camera_streaming(int index);
     void get_stream_list(std::vector<int> *list);

  };// end of class Services

}// cam_v4l2__pub
#endif // SERVICES_H
