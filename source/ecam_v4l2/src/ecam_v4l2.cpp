#include "ecam_v4l2/V4l2.h"
#include "ecam_v4l2/device.h"
#include "ecam_v4l2/services.h"
#include "nodelet/nodelet.h"

// Main function
int main(int argc, char ** argv)
{
  //local variables declaration
  int camera_count;

// node initialization
  ros::init(argc,argv,"cam_v4l2_node");
  ros::NodeHandle node_handle;
// Creating object for class Devices
  ecam_v4l2::Devices dev;

// Calling list_devices to enumerate all the devices connected to the system
  if(dev.list_devices()<0){
    // if no devices are connected, node is shutdown
    ros::shutdown();
  }

  dev.get_camera_count(&camera_count); // to get the total number of cameras connected
// Creating vector of objects for class Camera
  std::vector<ecam_v4l2::Camera> cam;

// Declaration of publishers for all cameras connected.
  ros::Publisher publisher[camera_count];
// Creating obj for each cameras and adding to the vector

  for(int cnt=0;cnt<camera_count;cnt++){
    ecam_v4l2::Camera obj;
    cam.push_back(obj);

    std::string cam_name;
    dev.get_camera_name(cnt,&cam_name);
// Publishing all the camera names as topics.
    publisher[cnt] = node_handle.advertise<ecam_v4l2::image>(cam_name.c_str(), 1);
  }
// Creating object for class Services
  ecam_v4l2::Services srv(dev,cam);
// Starting the server for all the Services provided by the Publisher.
  ros::ServiceServer Choose_device_server = node_handle.advertiseService("ChooseDevice",&ecam_v4l2::Services::onCameraChange,&srv);
  ros::ServiceServer query_control_server = node_handle.advertiseService("QueryControl",&ecam_v4l2::Services::isControlAvailable,&srv);
  ros::ServiceServer control_server = node_handle.advertiseService("SetControl",&ecam_v4l2::Services::onSetControl,&srv);
  ros::ServiceServer enum_format_server = node_handle.advertiseService("EnumerateFormat",&ecam_v4l2::Services::onEnumFormat,&srv);
  ros::ServiceServer format_set_server = node_handle.advertiseService("SetFormat",&ecam_v4l2::Services::onFormatChange,&srv);

  std::vector<int> stream_list;
  while (ros::ok())                       // Until Ctrl+C is pressed.
  {
        stream_list.clear();
        srv.get_stream_list(&stream_list);         // list of cameras selected in multiple subscribers
        for (auto cnt = stream_list.begin(); cnt != stream_list.end(); ++cnt)
        {

              if(cam[*cnt].isStreamOn()){              // checking whether the camera is streamed on.

                  ecam_v4l2::image image;

                  if(cam[*cnt].capture(&image)!=FAILURE) // Capturing image from the camera.
                  {
                      publisher[*cnt].publish(image);    // Publishing the image captured from the camera
                  }
              }
        }
        ros::spinOnce();
  }
  return SUCCESS;
} // end of main
