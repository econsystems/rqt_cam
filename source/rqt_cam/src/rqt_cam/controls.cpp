#include <rqt_cam/controls.h>

namespace rqt_cam{





  /************************************************************************************************************
  *  Name	:	select_camera.
  *  Parameter1 : std::string cam_name - name of the camera selected.
  *  Parameter2 :bool shutdown - flag which denotes whether to stream on or stream off the camera.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used to select the camera.
  ************************************************************************************************************/
  int Controls::select_camera(std::string cam_name,bool shutdown)
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient Choose_device_client = node_handle.serviceClient<rqt_cam::camera>("ChooseDevice");
    rqt_cam::camera srv;
    srv.request.cam_name = cam_name.c_str();
    srv.request.shutdown= shutdown;
    ros::service::waitForService("ChooseDevice");
    if(!Choose_device_client.call(srv)){
      ROS_ERROR("Failed to call service Choose_device");
      return FAILURE;
    }
    camera_name=cam_name;
    return SUCCESS;
  }





  /************************************************************************************************************
  *  Name	:	query_control.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used to query all the controls supported by the camera.
  ************************************************************************************************************/
  int Controls::query_control()
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient query_control_client = node_handle.serviceClient<rqt_cam::query_control>("QueryControl");
    rqt_cam::query_control qctrl;

    do{
      ros::service::waitForService("QueryControl");
      qctrl.request.id=V4L2_CTRL_FLAG_NEXT_CTRL;
      qctrl.request.cam_name=camera_name;
      if(query_control_client.call(qctrl)){
        Controls::queryctrl obj;
          obj.id=qctrl.response.id;
          obj.type=qctrl.response.type;
          obj.name = qctrl.response.name;
          obj.default_value= qctrl.response.default_value;
          obj.cur_value = qctrl.response.cur_value;
        if(qctrl.response.type==V4L2_CTRL_TYPE_INTEGER){
          obj.minimum =qctrl.response.minimum;
          obj.maximum =qctrl.response.maximum;
          obj.step=qctrl.response.step;
          obj.default_value= qctrl.response.default_value;
          obj.cur_value = qctrl.response.cur_value;
        }else if(qctrl.response.type==V4L2_CTRL_TYPE_MENU || qctrl.response.type==V4L2_CTRL_TYPE_INTEGER_MENU){
          obj.minimum =qctrl.response.minimum;
          obj.maximum =qctrl.response.maximum;
        }
        ctrl.push_back(obj);
      }else {
          if(ctrl.size()==0){
            ROS_ERROR("Failed to call service Query_control");
            return FAILURE;
          }
          break;
      }
    }while(1);

    return ctrl.size();
  }





  /************************************************************************************************************
  *  Name	:	set_control.
  *  Parameter1 : int value - Control value.
  *  Parameter2 : uint32_t id - Control ID.
  *  Parameter3 : int *cur_value- To store the current value.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used to set the UVC control.
  ************************************************************************************************************/
  int Controls::set_control(uint32_t id, int value)
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient control_client = node_handle.serviceClient<rqt_cam::set_control>("SetControl");
    rqt_cam::set_control sctrl;
    sctrl.request.cam_name=camera_name;
    sctrl.request.id =id;
    sctrl.request.value = value;
    ros::service::waitForService("SetControl");
    if(!control_client.call(sctrl)){
      ROS_ERROR("Failed to call service control");
      return FAILURE;
    }
    return SUCCESS;
  }





  /************************************************************************************************************
  *  Name	:	enum_format.
  *  Parameter1 : int type - This denotes whether to enumerate pixelformat or resolution or framerate.
                  The possible values are:
                  PIX_FORMAT                    1
                  RESOLUTION                    2
                  FPS                           3
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used to enumerate the pixelformat, resolution,framerates
  supported by the camera.
  ************************************************************************************************************/
  int Controls::enum_format(int type,std::string pix_fmt,int width,int height)
  {
    int index;
    ros::NodeHandle node_handle;
    ros::ServiceClient enum_format_client = node_handle.serviceClient<rqt_cam::enum_format>("EnumerateFormat");
    rqt_cam::enum_format fmt_msg;
    fmt_msg.request.cam_name= camera_name;
    switch (type) {
      case PIX_FORMAT:
        formats.clear();
        fmt_msg.request.type = PIX_FORMAT;
        ros::service::waitForService("EnumerateFormat");
        if(enum_format_client.call(fmt_msg)){
          for (index=0 ;index< fmt_msg.response.str.size(); index++){
            formats.push_back(fmt_msg.response.str[index]);
          }
        }else {
          ROS_ERROR("Failed to call service Format_setting");
          break;
        }
      case RESOLUTION:
        resolutions.clear();
        fmt_msg.request.type = RESOLUTION;
        fmt_msg.request.pix_fmt = pix_fmt;
        ros::service::waitForService("EnumerateFormat");
        if(enum_format_client.call(fmt_msg)){
          for (index=0 ;index< fmt_msg.response.str.size(); index++){
            resolutions.push_back(fmt_msg.response.str[index]);
          }
        }else {
          ROS_ERROR("Failed to call service Format_setting");
          break;
        }
      case FPS:
        framerates.clear();
        fmt_msg.request.type = FPS;
        fmt_msg.request.pix_fmt = pix_fmt;
        fmt_msg.request.height = height;
        fmt_msg.request.width = width;
        ros::service::waitForService("EnumerateFormat");
        if(enum_format_client.call(fmt_msg)){
          for (index=0 ;index< fmt_msg.response.str.size(); index++){
            framerates.push_back(fmt_msg.response.str[index]);
          }
        }else {
          ROS_ERROR("Failed to call service Format_setting");
          break;
        }
      default:
        break;
    }
    return SUCCESS;
  }





  /************************************************************************************************************
  *  Name	:	set_format.
  *  Parameter1 : std::string format- name of the format.
  *  Parameter2 : int width - Width of the image.
  *  Parameter3 : int height - Height of the image.
  *  Parameter4 : int numerator- Numerator of the framerate.
  *  Parameter5 : int denominator- Denominator of the framerate.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used to set the format, resolution and framerate.
  ************************************************************************************************************/
  int Controls::set_format(std::string format,int width,int height,int numerator, int denominator)
  {
    int index;
    ros::NodeHandle node_handle;
    ros::ServiceClient set_format_client = node_handle.serviceClient<rqt_cam::set_format>("SetFormat");
    rqt_cam::set_format set_fmt;
    set_fmt.request.cam_name=camera_name;
    set_fmt.request.format = format;
    set_fmt.request.width = width;
    set_fmt.request.height = height;
    set_fmt.request.numerator = numerator;
    set_fmt.request.denominator = denominator;
    ros::service::waitForService("SetFormat");
    if(!set_format_client.call(set_fmt)){
      ROS_ERROR("Failed to call service Format_setting");
      return FAILURE;
    }
    return SUCCESS;
  }






  /************************************************************************************************************
  *  Name	:	get_value.
  *  Parameter1 : int index - Index of the control.
  *  Returns int - requested value of the control.
  *  Description	:   This function is to get the requested value of the control.
  ************************************************************************************************************/
  int Controls::get_value(int index,int type)
  {

    switch (type) {
      case MIN_VAL:
        return ctrl[index].minimum;
      case MAX_VAL:
        return ctrl[index].maximum;
      case DEF_VAL:
        return ctrl[index].default_value;
      case CUR_VAL:
        return ctrl[index].cur_value;
      case STEP_VAL:
        return ctrl[index].step;
      case CTRL_ID:
        return ctrl[index].id;
      default:
        break;
    }

  }





  /************************************************************************************************************
  *  Name	:	get_control_name.
  *  Parameter1 : int index - Index of the control.
  *  Parameter2 : std::string *control_name - Name of the control.
  *  Description	:   This function is to get the Name of the control.
  ************************************************************************************************************/
  void Controls::get_control_name(int index,std::string *control_name)
  {
    *control_name = ctrl[index].name;
  }





  /************************************************************************************************************
  *  Name	:	get_list.
  *  Parameter1 : int type - This denotes whether to get list of pixelformat or resolution or framerate.
                  The possible values are:
                    PIX_FORMAT                    1
                    RESOLUTION                    2
                    FPS                           3
  *  Returns	:	std::vector<std::string> - requested list.
  *  Description	:   This function is get the list of all supported formats or resolutions or framerates.
  ************************************************************************************************************/
  std::vector<std::string> Controls::get_list(int type)
  {
    switch (type) {
      case PIX_FORMAT:
        return formats;
      case RESOLUTION:
        return resolutions;
      case FPS:
        return framerates;
    }
  }





  /************************************************************************************************************
  *  Name	:	clear_controls_list.
  *  Description	:   This function is to clear the list of controls which is already stored.
  ************************************************************************************************************/
  void Controls::clear_controls_list()
  {
    ctrl.clear();
  }


} // end of namespace rqt_cam
