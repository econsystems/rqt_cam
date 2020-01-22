#ifndef DEVICE_H
#define DEVICE_H
#include <libudev.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <ros/ros.h>
#define USB 1
#define MIPI 2
#define FAILURE                 -1
#define SUCCESS                  0
#define V4L2_CAP_META_CAPTURE    0x00800000


namespace ecam_v4l2
{

  class Devices
  {
  private:
      std::vector<std::string>device_node_name;
      std::vector<std::string>camera_name;
      bool get_product_name(std::string &dev_node);
      int check_for_valid_videonode(std::string &dev_node);
      int check_camera_type(std::string &dev_node,std::string *productName,int *type);
  public:
      int list_devices();
      void get_camera_count(int *camera_count);
      void get_device_node_name(int index,std::string *dev_node_name);
      void get_camera_name(int index,std::string *cam_name);
  };
} // namespace ecam_v4l2


#endif
