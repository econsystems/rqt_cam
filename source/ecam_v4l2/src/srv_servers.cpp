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
#include  "ecam_v4l2/srv_servers.h"
#include  "ecam_v4l2/camera.h"
#include  "ecam_v4l2/query_control.h"
#include  "ecam_v4l2/set_control.h"

namespace ecam_v4l2
{


  // Constructor function of class Services
  Services::Services(Devices &Devices_obj ,std::vector<Camera>& Camera_obj):
            dev(Devices_obj) ,cam(Camera_obj)
  {
    memset(&queryctrl, 0, sizeof(queryctrl));
    memset(&querymenu, 0, sizeof(querymenu));
  }

  /*****************************************************************************
  *  Name	:	get_camera_index.
  *  Parameter1 : std::string cam_name - Name of the camera.
  *  Parameter2 : int *current_device - index of the camera in objects list.
  *  Returns	:
  *  			True	- If the cam_name is present.
  *       False - If the cam_name is not present.
  *  Description	:   This function is to get the index of given cam_name
  *                   in the objects list.
  *****************************************************************************/
  bool Services::get_camera_index(std::string cam_name,int *current_device)
  {
    for (int cnt = 0; cnt != cam.size(); ++cnt) {
        if(cam_name==cam[cnt].get_camera_name()){
          *current_device = cnt;
          return true;
        }
    }
    *current_device=-1;
    return false;
  }

  /*****************************************************************************
  *  Name	:	compareList.
  *  Parameter1 : std::vector<std::string> prev_list - Camera list which is
  *               previously enumerated.
  *  Parameter2 : std::vector<std::string> cur_list - Camera list which is
  *               currently enumerated.
  *  Returns	:
  *  			True	- If both the list are same.
  *       False - If both the list are not same.
  *  Description	:   This function is to check whether any new camera is enumerated.
  *****************************************************************************/
  bool Services::compareList(std::vector<std::string> prev_list,
                             std::vector<std::string> cur_list)
  {
    if(prev_list.size()!=cur_list.size()){
      return false;
    }else{
      for (int cnt = 0; cnt != cur_list.size(); ++cnt) {
          if(cur_list[cnt].compare(prev_list[cnt])){
            return false;
          }
      }
    }
    return true;
  }

  /*****************************************************************************
  *  Name	:	onEnumDevice.
  *  Parameter1 : ecam_v4l2::camera::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::camera::Response &res - Service Response
  *  Returns	:
  *  			True	- If new camera/s are/is enumerated.
  *       False - If no new camera is enumerated.
  *  Description	:   This function is callback function which is called when
  *                   the topic combobox is pressed.
  *****************************************************************************/
  bool Services::onEnumDevice(ecam_v4l2::camera::Request &req,
                              ecam_v4l2::camera::Response &res)
  {
    int old_count,new_count,index;
    std::vector<std::string>prev_list;
    std::vector<std::string>cur_list;
    std::string cam_name,dev_node;
    ros::NodeHandle node_handle;
    std::vector<std::string>::iterator it;
    old_count = dev.get_camera_count();
    prev_list= dev.get_list();
    dev.list_devices();
    new_count = dev.get_camera_count();
    cur_list = dev.get_list();
    // if list before enumeration and after enumeration is same.
    if(compareList(prev_list,cur_list)){
      return false;
    }else{
        /* if any camera is removed, we stop publishing the topic
           and clear the camera object.*/
        for (int cnt = 0; cnt < prev_list.size(); ++cnt)
        {
          it = std::find(cur_list.begin(), cur_list.end(),prev_list[cnt]);
          if (it == cur_list.end()){
            if(get_camera_index(prev_list[cnt],&index)){
              stop_publisher(publisher[index],index);
            }
          }
        }
          /* if any new camera is connected, we are creating camera object and
             start publishing the topic.*/
          for(int cnt=0;cnt<new_count;cnt++)
          {
            dev.get_camera(cnt,&cam_name,&dev_node);
            // checking whether the camera is already enumerated.
            if(get_camera_index(cam_name,&index)==false){ //if not it is added.
              ecam_v4l2::Camera obj;
              obj.set_camera_name(cam_name,dev_node);
              ros::Publisher pub;
              pub = node_handle.advertise<ecam_v4l2::image>(cam_name.c_str(),1);
              cam.push_back(obj);
              publisher.push_back(pub);
            }
          }
    }// end of else
    return true;
  }


  /*****************************************************************************
  *  Name	:	onCameraChange.
  *  Parameter1 : ecam_v4l2::camera::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::camera::Response &res - Service Response
  *  Returns	:
  *  			True	- If the current_device is changed
  *
  *  Description	:   This function is callback function which is called when
                      the topic is changed in the subscriber's UI.
  *****************************************************************************/
  bool Services::onCameraChange(ecam_v4l2::camera::Request &req,
                                ecam_v4l2::camera::Response &res)
  {
    int current_device;
    // getting the device index for the requested camera name.
    if(get_camera_index(req.cam_name,&current_device)==false){
      return false;
    }
    memset(&queryctrl, 0, sizeof(queryctrl));
    memset(&querymenu, 0, sizeof(querymenu));
    if(req.shutdown){   // if shutdown request has come
        if(cam[current_device].sub_count()<0){
          cam[current_device].stream_off();
          cam[current_device].clean_buffer();
          cam[current_device].close_device();
          // removing the camera from streaming list
          remove_camera_streaming(req.cam_name);
      }
      ROS_INFO("%s camera streamed off!",req.cam_name.c_str());
      return true;
    }else{
      // checking whether the camera is not streaming.
      if(isNotStreaming(req.cam_name)){
        add_camera_streaming(req.cam_name);
      }else{
        cam[current_device].set_init(true);
        return false;
      }
      std::string dev_node_name="/dev/";
      cam[current_device].get_node_name(&dev_node_name);
      cam[current_device].open_device(dev_node_name.c_str());
      cam[current_device].cam_init();
    }

    return true;
  }




  /*****************************************************************************
  *  Name	:	isControlAvailable.
  *  Parameter1 : ecam_v4l2::query_control::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::query_control::Response &res - Service Response
  *  Returns	:
  *  			True	- If the control is available.
  *       False - If the VIDIOC_QUERYCTRL or V4L2_CTRL_FLAG_DISABLED gets failed.
  *  Description	:   This function is callback function which is called
  *                   to check whether particular control is available or not.
  *****************************************************************************/
  bool Services::isControlAvailable(ecam_v4l2::query_control::Request &req,
                                    ecam_v4l2::query_control::Response &res)
  {
    int current_device;
    // getting the device index for the requested camera name.
    if(get_camera_index(req.cam_name,&current_device)==false){
      return false;
    }
    if(req.reqtype==CTRL_TYPE){
      queryctrl.id |= req.id;
      if(0 == cam[current_device].xioctl(VIDIOC_QUERYCTRL, &queryctrl)) {
        if (!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {

          if(!check_valid_control(queryctrl.id)){
            return true;
          }
          res.id=queryctrl.id;
          res.type=queryctrl.type;
          res.name = std::string((char*)queryctrl.name);
          res.default_value= queryctrl.default_value;
          if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER){
            res.minimum =queryctrl.minimum;
            res.maximum =queryctrl.maximum;
            res.step=queryctrl.step;
          }else if(queryctrl.type == V4L2_CTRL_TYPE_MENU||
                   queryctrl.type==V4L2_CTRL_TYPE_INTEGER_MENU){
            res.minimum =queryctrl.minimum;
            res.maximum =queryctrl.maximum;
          }
          struct v4l2_control ctrl;
          memset(&ctrl,0,sizeof(ctrl));
          ctrl.id=queryctrl.id;

          if(0 == cam[current_device].xioctl(VIDIOC_G_CTRL, &ctrl)) {
            res.cur_value = ctrl.value;
            return true;
          }else{
            ROS_ERROR("Getting current value for %s control is failed",
                       (char*)queryctrl.name);
            ROS_WARN("Setting %s control to default_value",
                       (char*)queryctrl.name);
            res.cur_value = queryctrl.default_value;
            return true;
          }
        }
      }
    }else if(req.reqtype==MENU_TYPE){
      querymenu.id=req.id;
      querymenu.index =req.index;
      if(0 == cam[current_device].xioctl(VIDIOC_QUERYMENU, &querymenu)) {
        res.id=querymenu.id;
        res.name = std::string((char*)querymenu.name);
        res.default_value = querymenu.value;
        return true;
      }
    }else if(req.reqtype==GET_CTRL){
      struct v4l2_control ctrl;
      memset(&ctrl,0,sizeof(ctrl));
      ctrl.id=req.id;
      if(0 == cam[current_device].xioctl(VIDIOC_G_CTRL, &ctrl)) {
        res.cur_value = ctrl.value;
        return true;
      }
    }
    return false;
  }




  /*****************************************************************************
  *  Name	:	onSetControl.
  *  Parameter1 : ecam_v4l2::control::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::control::Response &res - Service Response
  *  Returns	:
  *  			True	- If the control is set successfully.
  *       False - if the control is not set.
  *  Description:This function is callback function which is used to set control.
  *****************************************************************************/
  bool Services::onSetControl(ecam_v4l2::set_control::Request &req,
                              ecam_v4l2::set_control::Response &res)
  {
    int current_device;
    // getting the device index for the requested camera name.
    if(get_camera_index(req.cam_name,&current_device)==false){
      return false;
    }
    struct v4l2_control ctrl;
    ctrl.id = req.id;
    ctrl.value = req.value;
    if(0 == cam[current_device].xioctl(VIDIOC_S_CTRL, &ctrl)) {
      return true;
    }
    return false;
  }




  /*****************************************************************************
  *  Name	:	onEnumFormat.
  *  Parameter1 : ecam_v4l2::enum_format::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::enum_format::Response &res - Service Response
  *  Returns	:
  *  True	- If the format is enumerated.
  *  False - if camera name does not matches with the enumerated list of cameras.
  *  Description	:   This function is callback function which is used to
  *                   enumerate formats,resolution and framerates.
  *****************************************************************************/
  bool Services::onEnumFormat(ecam_v4l2::enum_format::Request &req,
                              ecam_v4l2::enum_format::Response &res)
  {
    int current_device,cnt;
    int height,width;
    int numerator,denominator;
    std::string pixelformat;
    std::stringstream temp_stream;
    std::string format;
    std::vector<std::string> list;
    // getting the device index for the requested camera name.
    if(get_camera_index(req.cam_name,&current_device)==false){
      return false;
    }
    if(cam[current_device].onInit()){
      cam[current_device].get_format(&pixelformat,&height,&width,&numerator,
                                     &denominator,req.type);
    }else{
      pixelformat = req.pix_fmt;
      height = req.height;
      width = req.width;
    }
    if(req.type==PIX_FORMAT){
      list.clear();
      cam[current_device].enum_pixelformat(&list,&res.cur_val);
      for (auto cnt = list.begin(); cnt != list.end(); ++cnt) {
            res.str.push_back(*cnt);
      }
    }else if(req.type==RESOLUTION){
      list.clear();
      cam[current_device].enum_resolution(pixelformat,&list,&res.cur_val);
      for (auto cnt = list.begin(); cnt != list.end(); ++cnt) {
            res.str.push_back(*cnt);
      }
    }else if(req.type==FPS){
      list.clear();
      cam[current_device].enum_framerate(pixelformat,width,height,&list,
                                         &res.cur_val);
      for (auto cnt = list.begin(); cnt != list.end(); ++cnt) {
            res.str.push_back(*cnt);
      }
    }else if(req.type==GET_FORMAT){
      cam[current_device].get_format(&pixelformat,&height,&width,&numerator,
                                     &denominator,req.type);
      cam[current_device].get_four_character_code(v4l2_fourcc(pixelformat[0],
                                                              pixelformat[1],
                                                              pixelformat[2],
                                                              pixelformat[3]),
                                                              &format);
      res.str.push_back(format);
      temp_stream.str("");    // clearing temp_stream
      temp_stream<<width;
      temp_stream<<"x";
      temp_stream<<height;
      res.str.push_back(temp_stream.str());
      temp_stream.str("");    // clearing temp_stream
      temp_stream<<(double)denominator/numerator;
      temp_stream<<" FPS";
      res.str.push_back(temp_stream.str());
    }
    return true;
  }




  /*****************************************************************************
  *  Name	:	onFormatChange.
  *  Parameter1 : ecam_v4l2::format::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::format::Response &res - Service Response
  *  Returns	:
  *  True	- If the format is changed.
  *  False - if camera name does not matches with the enumerated list of cameras.
  *  Description	:   This function is callback function which is used
  *                   to change the format.
  *****************************************************************************/
  bool Services::onFormatChange(ecam_v4l2::set_format::Request &req,
                                ecam_v4l2::set_format::Response &res)
  {
    int current_device;
    // getting the device index for the requested camera name.
    if(get_camera_index(req.cam_name,&current_device)==false){
      return false;
    }
    cam[current_device].stream_off();
    cam[current_device].clean_buffer();
    cam[current_device].set_format(req.format,req.width,req.height);
    cam[current_device].set_framerate(req.numerator, req.denominator);
    cam[current_device].request_buffer();
    cam[current_device].stream_on();
    return true;
  }






  /*****************************************************************************
  *  Name	:	isNotStreaming
  *  Parameter1 : std::string- Name of the camera.
  *  Returns	:
  *  			True	- If the camera is not in streaming list.
  *       False - If the camera is already present in streaming list.
  *  Description	:   This function is to check whether the camera is
  *                   already present in the streaming list.
  *****************************************************************************/
  bool Services::isNotStreaming(std::string name)
  {
    for (auto cnt = stream_list.begin(); cnt != stream_list.end(); ++cnt) {
        if (*cnt == name) {
          return false;
        }
    }
    return true;
  }





  /*****************************************************************************
  *  Name	:	check_valid_control
  *  Parameter1 : int controlid - control id
  *  Returns	:
  *  			True	- If the control name  is a valid.
  *       False - If the control name  is a invalid.
  *  Description	: This function is to check whether the control name is valid.
  *****************************************************************************/
  bool Services::check_valid_control(int controlid)
  {
    if(controlid ==V4L2_CID_USER_CLASS||
       controlid==V4L2_CID_CAMERA_CLASS||
       controlid==V4L2_CID_ROI_EXPOSURE||
       controlid==V4L2_CID_ROI_WINDOW_SIZE||
       controlid >= V4L2_CID_CUSTOM_CONTROLS){
      return false;
    }
    return true;
  }





  /*****************************************************************************
  *  Name	:	add_camera_streaming
  *  Parameter1 : std::string- Name of the camera.
  *  Description	:   This function is to add camera to streaming list.
  *****************************************************************************/

  void Services::add_camera_streaming(std::string name)
  {
    stream_list.push_back(name);
  }




  /*****************************************************************************
  *  Name	:	remove_camera_streaming
  *  Parameter1 : std::string- Name of the camera.
  *  Description	:   This function is to remove camera from streaming list.
  *****************************************************************************/
  void Services::remove_camera_streaming(std::string name)
  {
    for (auto cnt = stream_list.begin(); cnt != stream_list.end(); ++cnt) {
        if (*cnt == name) {
            stream_list.erase(cnt);
            cnt--;
            break;
        }
    }
  }




  /*****************************************************************************
  *  Name	:	stop_publisher
  *  Parameter1 : ros::Publisher pub - publsiher which needs to be stopped.
  *  Parameter2 : int index - index of the camera in objects list.
  *  Description	:   This function is to stop publishing the given publisher
  *                   and clear the camera object.
  *****************************************************************************/
  void Services::stop_publisher(ros::Publisher pub,int index)
  {
    // removing the camera from streaming list
    remove_camera_streaming(cam[index].get_camera_name());
    // removing the camera name from the enumerated list.
    dev.remove_camera(cam[index].get_camera_name());
    cam[index].munmap_buffers();               // clearing the allocated buffer.
    cam.erase(cam.begin()+index);              // erasing object from the list.
    pub.shutdown();                            // shutting down the publsiher.
    publisher.erase(publisher.begin()+index);  // erasing publisher from list.
  }


  /*****************************************************************************
  *  Name	:	get_stream_list
  *  Parameter1 : std::vector<int> *list - streaming camera list
  *  Description	:   This function is to get the streaming camera list.
  *****************************************************************************/
  void Services::get_stream_list(std::vector<std::string> *list)
  {
    *list=stream_list;
  }


}//namespace ecam_v4l2
