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
#include "ecam_v4l2/V4l2.h"
#include "ecam_v4l2/device.h"
#include "ecam_v4l2/srv_servers.h"
#include "nodelet/nodelet.h"

// Main function
int main(int argc, char ** argv)
{
  //local variables declaration
  int camera_count,ret,current_device;
  std::vector<std::string> stream_list;

// node initialization
  ros::init(argc,argv,"cam_v4l2_node");
  ros::NodeHandle node_handle;
// Creating object for class Devices
  ecam_v4l2::Devices dev;

// Calling list_devices to enumerate all the devices connected to the system
  dev.list_devices();

// to get the total number of cameras connected
  camera_count = dev.get_camera_count();
// Creating vector of objects for class Camera
  std::vector<ecam_v4l2::Camera> cam;


// Creating obj for each cameras and adding to the vector
  ecam_v4l2::Services srv(dev,cam);

  for(int cnt=0;cnt<camera_count;cnt++){
    ecam_v4l2::Camera obj;

    std::string cam_name,dev_node;
    dev.get_camera(cnt,&cam_name,&dev_node);
    obj.set_camera_name(cam_name,dev_node);
    cam.push_back(obj);
    ros::Publisher pub;
// Publishing all the camera names as topics.
    pub = node_handle.advertise<ecam_v4l2::image>(cam_name.c_str(), 1);
    // pub.shutdown();
    srv.publisher.push_back(pub);
  }

// Starting the server for all the Services provided by the Publisher.
  ros::ServiceServer Choose_device_server = node_handle.advertiseService
                    ("ChooseDevice",&ecam_v4l2::Services::onCameraChange,&srv);

  ros::ServiceServer query_control_server = node_handle.advertiseService
                    ("QueryControl",&ecam_v4l2::Services::isControlAvailable,&srv);

  ros::ServiceServer control_server = node_handle.advertiseService
                    ("SetControl",&ecam_v4l2::Services::onSetControl,&srv);

  ros::ServiceServer enum_format_server = node_handle.advertiseService
                    ("EnumerateFormat",&ecam_v4l2::Services::onEnumFormat,&srv);

  ros::ServiceServer format_set_server = node_handle.advertiseService
                    ("SetFormat",&ecam_v4l2::Services::onFormatChange,&srv);

  ros::ServiceServer Enum_device_server = node_handle.advertiseService
                    ("EnumerateDevice",&ecam_v4l2::Services::onEnumDevice,&srv);


  while (ros::ok())                                  // Until Ctrl+C is pressed.
  {
        stream_list.clear();
        // list of cameras selected in multiple subscribers
        srv.get_stream_list(&stream_list);
        for (auto cnt = stream_list.begin(); cnt != stream_list.end(); ++cnt)
        {
              srv.get_camera_index(*cnt,&current_device);
              // checking whether the camera is streamed on.
              if(cam[current_device].isStreamOn()){
                  ecam_v4l2::image image;
                  // Capturing image from the camera.
                  ret=cam[current_device].capture(&image);
                  if(ret==SUCCESS) // if capture is successful.
                  {
                      // Publishing the image captured from the camera
                      srv.publisher[current_device].publish(image);

                  }else if(ret==DEV_UNPLUGGED){// If device is unplugged.
                      /* publishing empty image to notify the subscriber
                         that device is unplugged.*/
                      srv.publisher[current_device].publish(image);
                      // stop publishing the topic
                      srv.stop_publisher(srv.publisher[current_device],
                                                       current_device);
                  }
              }
        }
        ros::spinOnce();
  }
  return SUCCESS;
} // end of main
