#include  "ecam_v4l2/services.h"
#include  "ecam_v4l2/camera.h"
#include  "ecam_v4l2/query_control.h"
#include  "ecam_v4l2/set_control.h"

namespace ecam_v4l2
{


  // Constructor function of class Services
  Services::Services(Devices &Devices_obj ,std::vector<Camera>& Camera_obj): dev(Devices_obj) ,cam(Camera_obj)
  {
    memset(&queryctrl, 0, sizeof(queryctrl));
  }




  /************************************************************************************************************
  *  Name	:	onCameraChange.
  *  Parameter1 : ecam_v4l2::camera::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::camera::Response &res - Service Response
  *  Returns	:
  *  			True	- If the current_device is changed
  *
  *  Description	:   This function is callback function which is called when the topic is changed in the subscriber's UI.
  ************************************************************************************************************/
  bool Services::onCameraChange(ecam_v4l2::camera::Request &req,ecam_v4l2::camera::Response &res)
  {
    int current_device;
    // getting the device index for the requested camera name.
    if(get_current_device(req.cam_name,&current_device)==false){

      return false;
    }
    if(req.shutdown){   // if shutdown request has come
      cam[current_device].stream_off();
      cam[current_device].clean_buffer();
      cam[current_device].close_device();
      remove_camera_streaming(current_device); // removing the camera from streaming list
      return true;
    }else{
      if(isNotStreaming(current_device)){  // checking whether the camera is not streaming.
        add_camera_streaming(current_device);
      }else{
        return false;
      }
      memset(&queryctrl, 0, sizeof(queryctrl));
      std::string dev_node_name="/dev/";
      dev.get_device_node_name(current_device,&dev_node_name);
      cam[current_device].open_device(dev_node_name.c_str());
      cam[current_device].init_format();
    }

    return true;
  }




  /************************************************************************************************************
  *  Name	:	isControlAvailable.
  *  Parameter1 : ecam_v4l2::query_control::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::query_control::Response &res - Service Response
  *  Returns	:
  *  			True	- If the control is available.
  *       False - If the VIDIOC_QUERYCTRL or V4L2_CTRL_FLAG_DISABLED gets failed.
  *  Description	:   This function is callback function which is called to check whether particular control is available or not.
  ************************************************************************************************************/
  bool Services::isControlAvailable(ecam_v4l2::query_control::Request &req,ecam_v4l2::query_control::Response &res)
  {
    int current_device;
    // getting the device index for the requested camera name.
    if(get_current_device(req.cam_name,&current_device)==false){

      return false;
    }

    queryctrl.id |= req.id;
    if(0 == cam[current_device].xioctl(VIDIOC_QUERYCTRL, &queryctrl)) {
      if (!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {

        res.id=queryctrl.id;
        res.type=queryctrl.type;
        res.name = std::string((char*)queryctrl.name);
        if(!check_valid_control(res.name)){
          return true;
        }
        res.default_value= queryctrl.default_value;
        if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER){
          res.minimum =queryctrl.minimum;
          res.maximum =queryctrl.maximum;
          res.step=queryctrl.step;
        }else if(queryctrl.type == V4L2_CTRL_TYPE_MENU|| queryctrl.type==V4L2_CTRL_TYPE_INTEGER_MENU){
          res.minimum =queryctrl.minimum;
          res.maximum =queryctrl.maximum;
        }
        struct v4l2_control ctrl;
        memset(&ctrl,0,sizeof(ctrl));
        ctrl.id=queryctrl.id;

        if(0 == cam[current_device].xioctl(VIDIOC_G_CTRL, &ctrl)) {
          res.cur_value = ctrl.value;
          return true;
        }
      }
    }
    return false;
  }




  /************************************************************************************************************
  *  Name	:	onSetControl.
  *  Parameter1 : ecam_v4l2::control::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::control::Response &res - Service Response
  *  Returns	:
  *  			True	- If the control is set successfully.
  *       False - if the control is not set.
  *  Description	:   This function is callback function which is used to set control.
  ************************************************************************************************************/
  bool Services::onSetControl(ecam_v4l2::set_control::Request &req,ecam_v4l2::set_control::Response &res)
  {
    int current_device;
    // getting the device index for the requested camera name.
    if(get_current_device(req.cam_name,&current_device)==false){

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




  /************************************************************************************************************
  *  Name	:	onEnumFormat.
  *  Parameter1 : ecam_v4l2::enum_format::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::enum_format::Response &res - Service Response
  *  Returns	:
  *  			True	- If the format is enumerated.
  *       False - if camera name does not matches with the enumerated list of cameras.
  *  Description	:   This function is callback function which is used to enumerate formats,resolution and framerates.
  ************************************************************************************************************/
  bool Services::onEnumFormat(ecam_v4l2::enum_format::Request &req,ecam_v4l2::enum_format::Response &res)
  {
    int current_device,cnt;
    int height,width;
    std::string pixelformat;

    std::stringstream temp_stream;
    std::string format;
    // getting the device index for the requested camera name.
    if(get_current_device(req.cam_name,&current_device)==false){
      return false;
    }
    if(cam[current_device].onInit()){
      cam[current_device].get_format(&pixelformat,&height,&width,req.type);
    }else{
      pixelformat = req.pix_fmt;
      height = req.height;
      width = req.width;
    }

    if(req.type==PIX_FORMAT){

      for (cnt=0 ;cnt< MAX_PIXELFORMAT_SUPPORT; cnt++){
        temp_stream.str("");    // clearing temp_stream
        cam[current_device].frmt.ffmt[cnt].index		= cnt;
        cam[current_device].frmt.ffmt[cnt].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if(0 == cam[current_device].xioctl(VIDIOC_ENUM_FMT, &cam[current_device].frmt.ffmt[cnt])) {
          cam[current_device].get_four_character_code(cam[current_device].frmt.ffmt[cnt].pixelformat,&format);
          res.str.push_back(format);
        }
        else{
          break;
        }
      }
    }else if(req.type==RESOLUTION){
      for (cnt=0 ;cnt< MAX_FRAME_SIZE_SUPPORT; cnt++){
        temp_stream.str("");    // clearing temp_stream
        cam[current_device].frmt.fsize[cnt].index		= cnt;
        cam[current_device].frmt.fsize[cnt].pixel_format	= v4l2_fourcc(pixelformat[0], pixelformat[1], pixelformat[2], pixelformat[3]);
        if(0 == cam[current_device].xioctl(VIDIOC_ENUM_FRAMESIZES, &cam[current_device].frmt.fsize[cnt])) {
          temp_stream<<cam[current_device].frmt.fsize[cnt].discrete.width;
          temp_stream<<"x";
          temp_stream<<cam[current_device].frmt.fsize[cnt].discrete.height;
          res.str.push_back(temp_stream.str());
        }
        else{
          break;
        }
      }
    }else if(req.type==FPS){
      for (cnt=0 ;cnt< MAX_FRMINVAL_SUPPORT; cnt++){
        temp_stream.str("");    // clearing temp_stream
        cam[current_device].frmt.frminterval.index	= cnt;
        cam[current_device].frmt.frminterval.width = width;
        cam[current_device].frmt.frminterval.height = height;
        cam[current_device].frmt.frminterval.pixel_format = v4l2_fourcc(pixelformat[0], pixelformat[1], pixelformat[2], pixelformat[3]);
        if(0 == cam[current_device].xioctl(VIDIOC_ENUM_FRAMEINTERVALS, &cam[current_device].frmt.frminterval)) {
          if(cam[current_device].frmt.frminterval.type == V4L2_FRMIVAL_TYPE_DISCRETE){
            cam[current_device].frmt.desc_frm_inval[cnt].numerator = cam[current_device].frmt.frminterval.discrete.numerator;
            cam[current_device].frmt.desc_frm_inval[cnt].denominator = cam[current_device].frmt.frminterval.discrete.denominator;
          }
          /*dividing denominator by numerator.
            for ex: (1) den-30, num- 1
                    30/1 = 30 FPS
                    (2) den-15, num -2
                    15/2 = 7.5 FPS
          */
          temp_stream<<(double)cam[current_device].frmt.desc_frm_inval[cnt].denominator/cam[current_device].frmt.desc_frm_inval[cnt].numerator;
          temp_stream<<" FPS";
          res.str.push_back(temp_stream.str());
        }
        else{
          break;
        }
      }
    }
    return true;
  }




  /************************************************************************************************************
  *  Name	:	onFormatChange.
  *  Parameter1 : ecam_v4l2::format::Request &req - Service request.
  *  Parameter2 :ecam_v4l2::format::Response &res - Service Response
  *  Returns	:
  *  			True	- If the format is changed.
  *       False - if camera name does not matches with the enumerated list of cameras.
  *  Description	:   This function is callback function which is used to change the format.
  ************************************************************************************************************/
  bool Services::onFormatChange(ecam_v4l2::set_format::Request &req,ecam_v4l2::set_format::Response &res)
  {
    int current_device;
    // getting the device index for the requested camera name.
    if(get_current_device(req.cam_name,&current_device)==false){
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




  /************************************************************************************************************
  *  Name	:	get_current_device
  *  Returns : bool . true
  *  Description	:   This function is to get the current_device.
  ************************************************************************************************************/

  bool Services::get_current_device(std::string name,int *index)
  {
    int camera_count,cnt=0;
    dev.get_camera_count(&camera_count);
    *index=-1;
    while(cnt<camera_count){
      std::string cam_name;
      dev.get_camera_name(cnt,&cam_name);
      if(name.compare(cam_name.c_str())==0){
        *index=cnt;
        return true;
      }
      cnt++;
    }
    return false;
  }





  /************************************************************************************************************
  *  Name	:	isNotStreaming
  *  Parameter1 : int index - camera index.
  *  Returns	:
  *  			True	- If the camera is not in streaming list.
  *       False - If the camera is already present in streaming list.
  *  Description	:   This function is to check whether the camera is already present in the streaming list.
  ************************************************************************************************************/
  bool Services::isNotStreaming(int index)
  {
    for (auto cnt = stream_list.begin(); cnt != stream_list.end(); ++cnt) {
        if (*cnt == index) {
          return false;
        }
    }
    return true;
  }





  /************************************************************************************************************
  *  Name	:	check_valid_control
  *  Parameter1 : std::string control_name - Name of the control
  *  Returns	:
  *  			True	- If the control name  is a valid.
  *       False - If the control name  is a invalid.
  *  Description	:   This function is to check whether the control name is valid or not.
  ************************************************************************************************************/
  bool Services::check_valid_control(std::string control_name)
  {
    if(control_name=="User Controls"||control_name=="Camera Controls"){
      return false;
    }
    return true;
  }





  /************************************************************************************************************
  *  Name	:	add_camera_streaming
  *  Parameter1 : int index - camera index.
  *  Description	:   This function is to add camera to streaming list.
  ************************************************************************************************************/

  void Services::add_camera_streaming(int index)
  {
    stream_list.push_back(index);
  }




  /************************************************************************************************************
  *  Name	:	remove_camera_streaming
  *  Parameter1 : int index - camera index.
  *  Description	:   This function is to remove camera from streaming list.
  ************************************************************************************************************/
  void Services::remove_camera_streaming(int index)
  {
    for (auto cnt = stream_list.begin(); cnt != stream_list.end(); ++cnt) {
        if (*cnt == index) {
            stream_list.erase(cnt);
            cnt--;
            break;
        }
    }
  }




  /************************************************************************************************************
  *  Name	:	get_stream_list
  *  Parameter1 : std::vector<int> *list - streaming camera list
  *  Description	:   This function is to get the streaming camera list.
  ************************************************************************************************************/
  void Services::get_stream_list(std::vector<int> *list)
  {
    *list=stream_list;
  }


}//namespace ecam_v4l2
