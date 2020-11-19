/*
 * The MIT License (MIT)
 *
 * ECAM_V4L2
 * Copyright (c) 2020 e-consystems
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef SERVICES_SERVERS_H
#define SERVICES_SERVERS_H
#include "ecam_v4l2/camera.h"
#include "ecam_v4l2/device.h"
#include "ecam_v4l2/query_control.h"
#include "ecam_v4l2/set_control.h"
#include "ecam_v4l2/set_format.h"
#include "ecam_v4l2/enum_format.h"
#include "ecam_v4l2/V4l2.h"
#include <sys/mman.h>
#include <string>
#include <vector>
#include <algorithm>
#define CTRL_TYPE                     7
#define MENU_TYPE                     8
#define GET_CTRL                      9
#define GET_FORMAT                    10
#define V4L2_CID_CUSTOM_CONTROLS      V4L2_CID_CAMERA_CLASS_BASE+50
#define V4L2_CID_ROI_WINDOW_SIZE      V4L2_CID_CAMERA_CLASS_BASE+36
#define V4L2_CID_ROI_EXPOSURE         V4L2_CID_CAMERA_CLASS_BASE+38

namespace ecam_v4l2
{
   class Services
   {
     private:
       Devices &dev; // container to hold obj of class Devices
       std::vector<Camera>& cam; // container to hold obj of class Camera
       struct v4l2_queryctrl queryctrl;
       struct v4l2_querymenu querymenu;
       std::vector<std::string> stream_list;// list of cameras which are streaming

       bool compareList(std::vector<std::string> prev_list,
                        std::vector<std::string> cur_list);

  	 public:
       // Declaration of publishers for all cameras connected.
       std::vector<ros::Publisher> publisher;

       Services(Devices &Devices_obj ,std::vector<Camera>& Camera_obj);

       bool onEnumDevice(ecam_v4l2::camera::Request &req,
                         ecam_v4l2::camera::Response &res);

       bool onCameraChange(ecam_v4l2::camera::Request &req,
                           ecam_v4l2::camera::Response &res);

       bool isControlAvailable(ecam_v4l2::query_control::Request &req,
                               ecam_v4l2::query_control::Response &res);

       bool onSetControl(ecam_v4l2::set_control::Request &req,
                         ecam_v4l2::set_control::Response &res);

       bool onEnumFormat(ecam_v4l2::enum_format::Request &req,
                         ecam_v4l2::enum_format::Response &res);

       bool onFormatChange(ecam_v4l2::set_format::Request &req,
                           ecam_v4l2::set_format::Response &res);

       bool get_current_device(std::string name,int *index);

       bool isNotStreaming(std::string name);

       bool check_valid_control(int controlid);

       void stop_publisher(ros::Publisher pub,int index);

       void add_camera_streaming(std::string name);

       void remove_camera_streaming(std::string name);

       void get_stream_list(std::vector<std::string> *list);

       bool get_camera_index(std::string cam_name,int *current_device);

    };// end of class Services

}// cam_v4l2__pub
#endif // SERVICES_H
