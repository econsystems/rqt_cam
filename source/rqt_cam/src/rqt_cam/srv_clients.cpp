/*
 * Copyright (c) 2020, e-consystems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the e-consystems nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <rqt_cam/srv_clients.h>

namespace rqt_cam{




  /*****************************************************************************
  *  Name	:	enum_device.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description:This function is serviceClient function used to enumerate the devices.
  *****************************************************************************/
  int Services::enum_device()
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient enum_device_client = node_handle.serviceClient
                                            <rqt_cam::camera>("EnumerateDevice");
    rqt_cam::camera srv;
    ros::service::waitForService("EnumerateDevice");
    if(!enum_device_client.call(srv)){
      ROS_WARN("No camera is newly enumerated");
      return FAILURE;
    }
    return SUCCESS;
  }




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
  int Services::select_camera(std::string cam_name,bool shutdown)
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient Choose_device_client = node_handle.serviceClient
                                              <rqt_cam::camera>("ChooseDevice");
    rqt_cam::camera srv;
    srv.request.cam_name = cam_name.c_str();
    srv.request.shutdown= shutdown;
    ros::service::waitForService("ChooseDevice");
    camera_name=cam_name;
    if(!Choose_device_client.call(srv)){
      ROS_ERROR("Failed to call service Choose_device");
      return FAILURE;
    }
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
  int Services::query_control()
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient query_control_client = node_handle.serviceClient
                                              <rqt_cam::query_control>("QueryControl");
    rqt_cam::query_control qctrl;

    do{
        ros::service::waitForService("QueryControl");
        qctrl.request.id=V4L2_CTRL_FLAG_NEXT_CTRL;
        qctrl.request.cam_name=camera_name;
        qctrl.request.reqtype = CTRL_TYPE;
        if(query_control_client.call(qctrl)){
          Services::queryctrl obj;
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
          }else if(qctrl.response.type==V4L2_CTRL_TYPE_MENU ||
                   qctrl.response.type==V4L2_CTRL_TYPE_INTEGER_MENU){
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
  /*****************************************************************************
  *  Name	:	v4l2_query_menu.
  *  Parameter1 : uint32_t id - Control ID.
  *  Parameter2 : int32_t minimum - minimum index of menu.
  *  Parameter2 : int32_t maximum - maximum index of menu.
  *  Parameter2 : QStringList &modes - list to store the menu.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used
  *                   to query the menu type control.
  *****************************************************************************/
  int Services::v4l2_query_menu(int32_t id,int32_t minimum,int32_t maximum,
                                QStringList &modes)
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient query_menu_client = node_handle.serviceClient
                                           <rqt_cam::query_control>("QueryControl");
    rqt_cam::query_control qmenu;
    int index;
    QString mode;
    for(index= minimum;index<=maximum;index++)
    {
      ros::service::waitForService("QueryControl");
      qmenu.request.id = id;
      qmenu.request.cam_name=camera_name;
      qmenu.request.reqtype = MENU_TYPE;
      qmenu.request.index=index;
      if(query_menu_client.call(qmenu)){
        mode = QString::fromStdString(qmenu.response.name);
        modes << mode;
      }
    }

    return SUCCESS;
  }



  /*****************************************************************************
  *  Name	:	set_control.
  *  Parameter1 : uint32_t id - Control ID.
  *  Parameter2 : int value - Control value.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used
  *                   to set the UVC control.
  *****************************************************************************/
  int Services::set_control(uint32_t id, int value)
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient control_client = node_handle.serviceClient
                                        <rqt_cam::set_control>("SetControl");
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




  /*****************************************************************************
  *  Name	:	get_control.
  *  Parameter1 : uint32_t id - Control ID.
  *  Parameter2 : int *cur_value- To store the current value.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used
  *                   to get the UVC control.
  *****************************************************************************/
  int Services::get_control(uint32_t id, int *cur_value)
  {
    ros::NodeHandle node_handle;
    ros::ServiceClient get_control_client = node_handle.serviceClient
                                            <rqt_cam::query_control>("QueryControl");
    rqt_cam::query_control gctrl;
    gctrl.request.cam_name=camera_name;
    gctrl.request.id =id;
    gctrl.request.reqtype = GET_CTRL;
    ros::service::waitForService("QueryControl");
    if(get_control_client.call(gctrl)){
      *cur_value =gctrl.response.cur_value;
      return SUCCESS;
    }

    return FAILURE;
  }






  /************************************************************************************************************
  *  Name	:	enum_format.
  *  Parameter1 : int type - This denotes whether to enumerate pixelformat
                             or resolution or framerate.
                  The possible values are:
                  PIX_FORMAT                    1
                  RESOLUTION                    2
                  FPS                           3
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used to enumerate
  *                   the pixelformat, resolution,framerates.
  ************************************************************************************************************/
  int Services::enum_format(int type,std::string *pix_fmt,int width,int height,
                            std::string *cur_res,std::string *cur_fps)
  {
    int index;
    char dummy_char;
    ros::NodeHandle node_handle;
    ros::ServiceClient enum_format_client = node_handle.serviceClient
                                            <rqt_cam::enum_format>("EnumerateFormat");
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
          *pix_fmt = fmt_msg.response.cur_val;
        }else {
          ROS_ERROR("Failed to call service Format_setting");
          break;
        }
      case RESOLUTION:
        resolutions.clear();
        fmt_msg.request.type = RESOLUTION;
        fmt_msg.request.pix_fmt = *pix_fmt;
        ros::service::waitForService("EnumerateFormat");
        if(enum_format_client.call(fmt_msg)){
          for (index=0 ;index< fmt_msg.response.str.size(); index++){
            resolutions.push_back(fmt_msg.response.str[index]);
          }
          *cur_res = fmt_msg.response.cur_val;
          std::istringstream temp1_stream(fmt_msg.response.cur_val);
          // Getting currently enumerated height and width.
          temp1_stream >>width >> dummy_char >> height;

        }else {
          ROS_ERROR("Failed to call service Format_setting");
          break;
        }

      case FPS:
        framerates.clear();
        fmt_msg.request.type = FPS;
        fmt_msg.request.pix_fmt = *pix_fmt;
        fmt_msg.request.height = height;
        fmt_msg.request.width = width;
        ros::service::waitForService("EnumerateFormat");
        if(enum_format_client.call(fmt_msg)){
          for (index=0 ;index< fmt_msg.response.str.size(); index++){
            framerates.push_back(fmt_msg.response.str[index]);
          }
          *cur_fps = fmt_msg.response.cur_val;
        }else {
          ROS_ERROR("Failed to call service Format_setting");
          break;
        }

      default:
        break;
    }
    return SUCCESS;
  }





  /*****************************************************************************
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
  *  Description	:   This function is serviceClient function used to set
  *                   the format, resolution and framerate.
  // **************************************************************************/
  int Services::set_format(std::string format,int width,int height,int numerator,
                           int denominator)
  {
    int index;
    ros::NodeHandle node_handle;
    ros::ServiceClient set_format_client = node_handle.serviceClient
                                           <rqt_cam::set_format>("SetFormat");
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





  /*****************************************************************************
  *  Name	:	get_format.
  *  Parameter1 : std::string format- name of the format.
  *  Parameter2 : std::string *resolution - resolution.
  *  Parameter3 : std::string *framerate - framerate.
  *  Parameter4 : int numerator- Numerator of the framerate.
  *  Parameter5 : int denominator- Denominator of the framerate.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is serviceClient function used to get
  *                   the format, resolution and framerate.
  *****************************************************************************/
  int Services::get_format(std::string *format,std::string *resolution,
                           std::string *framerate)
  {
    int index;
    ros::NodeHandle node_handle;
    ros::ServiceClient enum_format_client = node_handle.serviceClient
                                            <rqt_cam::enum_format>("EnumerateFormat");
    rqt_cam::enum_format fmt_msg;
    fmt_msg.request.cam_name= camera_name;
    fmt_msg.request.type = GET_FORMAT;
        ros::service::waitForService("EnumerateFormat");
        if(enum_format_client.call(fmt_msg)){
          *format=fmt_msg.response.str[0];
          *resolution=fmt_msg.response.str[1];
          *framerate=fmt_msg.response.str[2];
        }else {
          ROS_ERROR("Failed to call service Format_setting");
          return FAILURE;
        }

    return SUCCESS;
  }





  /*****************************************************************************
  *  Name	:	get_value.
  *  Parameter1 : int index - Index of the control.
  *  Returns int - requested value of the control.
  *  Description	:   This function is to get the requested value of the control.
  *****************************************************************************/
  void Services::get_value(int index,std::string *control_name,int *control_type,
                           int *control_id, int * min_val,int *max_val,
                           int * cur_val,int * step_val,int *def_val)
  {
        *control_name = ctrl[index].name;
        *control_type = ctrl[index].type;
        *control_id = ctrl[index].id;
        *min_val = ctrl[index].minimum;
        *max_val = ctrl[index].maximum;
        *cur_val = ctrl[index].cur_value;
        *step_val = ctrl[index].step;
        *def_val = ctrl[index].default_value;

  }






  /*****************************************************************************
  *  Name	:	get_list.
  *  Parameter1 : int type - This denotes whether to get list of pixelformat
  *                           or resolution or framerate.
                  The possible values are:
                    PIX_FORMAT                    1
                    RESOLUTION                    2
                    FPS                           3
  *  Returns	:	std::vector<std::string> - requested list.
  *  Description	:   This function is get the list of all supported formats
  *                   or resolutions or framerates.
  *****************************************************************************/
  std::vector<std::string> Services::get_list(int type)
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





  /*****************************************************************************
  *  Name	:	clear_controls_list.
  *  Description	:   This function is to clear the list of controls
  *                   which is already stored.
  *****************************************************************************/
  void Services::clear_controls_list()
  {
    ctrl.clear();
  }


} // end of namespace rqt_cam
