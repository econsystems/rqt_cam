#include "ecam_v4l2/device.h"

namespace ecam_v4l2
{



  /************************************************************************************************************
  *  Name	:	get_product_name
  *  Parameter1	:	dev_node- This is the video node name
  *  Returns	:	Function result depends on cndition available in the functions.
  *  Description	:   This function is to get the product Name of the camera.
  ************************************************************************************************************/
  bool Devices::get_product_name(std::string &dev_node)
  {
    int type;
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev,*pdev;
    std::string parameter ="video4linux";
    std::string search_video_device ="video4linux/";
    search_video_device+=dev_node;
    /* Create the udev object */
    udev = udev_new();
    if (!udev)  return false;

    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, parameter.c_str());
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    udev_list_entry_foreach(dev_list_entry, devices) {

      std::string path;

      path = udev_list_entry_get_name(dev_list_entry);

      dev = udev_device_new_from_syspath(udev, path.c_str());

      if(std::strstr(path.c_str(),search_video_device.c_str())!=NULL){
        std::string device_node = "/dev/";
        std::string productName;
        device_node+= dev_node;
        if(check_camera_type(device_node,&productName,&type)!=FAILURE){
          if(type==USB){
            pdev = udev_device_get_parent_with_subsystem_devtype(dev,"usb","usb_device");
            std::string serialno = udev_device_get_sysattr_value(pdev,"serial");
            productName+="_";  // To add '_' between camera name and serial no, ex: "See3CAM_130_1523C405"
            productName+=serialno;
            camera_name.push_back(productName);
          }else {
            //Replacing ('-',',',' ') with underscore, Since those characters cannot be used in topic name.
            std::replace( productName.begin(), productName.end(), '-', '_');
            std::replace( productName.begin(), productName.end(), ',', '_');
            std::replace( productName.begin(), productName.end(), ' ', '_');
            camera_name.push_back(productName);
          }
        }
      }
      else{
        udev_device_unref(dev);
        continue;
      }
      udev_device_unref(dev);
    }
    udev_enumerate_unref(enumerate);
    udev_unref(udev);
    return true;
  }




  /************************************************************************************************************
  *  Name	:	check_for_valid_videonode
  *  Parameter1	:	std::string &device_node - Name of the device node.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:  	This function is check whether the node is valid node.
  ************************************************************************************************************/
  int Devices::check_for_valid_videonode(std::string &dev_node)
  {
    int cam_fd;
    struct v4l2_capability 		cam_cap;
    if ((cam_fd = open(dev_node.c_str(), O_RDWR|O_NONBLOCK, 0)) < 0) {
      ROS_INFO("Can't open camera device ");
      return FAILURE;
    }
    /* Check if the device is capable of streaming */
    if(ioctl(cam_fd, VIDIOC_QUERYCAP, &cam_cap) < 0) {
      ROS_INFO(" VIDIOC_QUERYCAP failure");
      close(cam_fd);
      return FAILURE;
    }
    close(cam_fd);
    if (cam_cap.device_caps & V4L2_CAP_META_CAPTURE) {
      return FAILURE;
    }else {
      return SUCCESS;
    }
  }




  /************************************************************************************************************
  *  Name	:	check_camera_type
  *  Parameter1	:	std::string &device_node - Name of the device node.
  *  Parameter2 : std::string *productName - Name of the camera.
  *  Parameter3 : int *type - USB or MIPI.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:  	This function is check whether the camera is USB camera or MIPI camera.
  ************************************************************************************************************/
  int Devices::check_camera_type(std::string &dev_node,std::string *productName,int *type)
  {
    int cam_fd;
    std::string name;
    struct v4l2_capability cam_cap;
    const int type1 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if ((cam_fd = open(dev_node.c_str(), O_RDWR|O_NONBLOCK, 0)) < 0) {
      ROS_INFO("Can't open camera device ");
      return FAILURE;
    }
    /* Check if the device is capable of streaming */
    if(ioctl(cam_fd, VIDIOC_QUERYCAP, &cam_cap) < 0) {
      ROS_INFO(" VIDIOC_QUERYCAP failure");
      return FAILURE;
    }
    name=(char*)cam_cap.driver;
    *productName=(char*)cam_cap.card;
    if(name.compare("uvcvideo")==0){
      *type=USB;
    }else{
      *type=MIPI;
    }
    close(cam_fd);
    return SUCCESS;
  }





  /************************************************************************************************************
  *  Name	:	list_devices
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:  	This function enumerates the connected cameras and makes a list of device node names.
  ************************************************************************************************************/
  int Devices::list_devices()
  {
    DIR *video_node_dir;
    struct dirent *video_node;
    std::string video_node_name;
    video_node_dir = opendir("/sys/class/video4linux/");
    if(video_node_dir == NULL){
      ROS_INFO("No video streaming devices found ");
      return FAILURE;
    }
    else{
      ROS_INFO("\n==================== Video devices detected ====================");
      while((video_node = readdir(video_node_dir))!= NULL){
        if(video_node->d_type == DT_LNK){
          video_node_name = video_node->d_name;
          if(get_product_name(video_node_name)){
            std::string device_node = "/dev/";
            device_node+= video_node_name;
            if(check_for_valid_videonode(device_node)==0){
              device_node_name.push_back(video_node->d_name);
            }
          }
        }
      }
      if(device_node_name.size() == 0){    // if no devices are enumerated
        ROS_INFO("No video streaming devices found \n");
        closedir(video_node_dir);
        return FAILURE;
      }
      for(int i=0;i<device_node_name.size();i++){
        ROS_INFO("%d.%s\n",i+1,camera_name[i].c_str());
      }
      closedir(video_node_dir);
    }
    return SUCCESS;
  }




  /************************************************************************************************************
  *  Name	:	get_camera_count
  *  Parameter1 : int - Total number of connected camera.
  *  Description	:   This function is to get the total number of devices connected.
  ************************************************************************************************************/
  void Devices::get_camera_count(int *camera_count)
  {
    *camera_count =device_node_name.size();
  }




  /************************************************************************************************************
  *  Name	:	get_device_node_name.
  *  Parameter1 : int index - The index for which the device node is needed,
  *  Parameter2 : std::string *dev_node_name - Name of the device node.
  *  Description	:   This function is to get the device node name of the given index.
  ************************************************************************************************************/
  void Devices::get_device_node_name(int index,std::string *dev_node_name)
  {
    *dev_node_name += device_node_name[index];
  }



  /************************************************************************************************************
  *  Name	:	get_camera_name.
  *  Parameter1 : int index - The index for which the device node is needed,
  *  Parameter2 : std::string *cam_name -Name of the camera.
  *  Description	:   This function is to get the camera name of the given index.
  ************************************************************************************************************/
  void Devices::get_camera_name(int index,std::string *cam_name)
  {
    *cam_name=camera_name[index];
  }
} // namespace ecam_v4l2
