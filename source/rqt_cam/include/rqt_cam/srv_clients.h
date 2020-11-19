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
#ifndef SERVICES_CLIENTS_H
#define SERVICES_CLIENTS_H
#include "rqt_cam/query_control.h"
#include "rqt_cam/set_control.h"
#include "rqt_cam/set_format.h"
#include "rqt_cam/enum_format.h"
#include "rqt_cam/camera.h"
#include <QString>
#include <QSet>
#include <ros/ros.h>
#include <linux/videodev2.h>

#define FAILURE                      -1
#define SUCCESS                       0
#define PIX_FORMAT                    1
#define RESOLUTION                    2
#define FPS                           3
#define MIN_VAL                       1
#define MAX_VAL                       2
#define DEF_VAL                       3
#define CUR_VAL                       4
#define STEP_VAL                      5
#define CTRL_ID                       6
#define CTRL_TYPE                     7
#define MENU_TYPE                     8
#define GET_CTRL                      9
#define GET_FORMAT                    10
namespace rqt_cam{

  class Services
  {
    private:

      class queryctrl{
        public:
          uint32_t id;
          uint32_t type;
          std::string name;
          int32_t minimum;
          int32_t maximum;
          int32_t step;
          int32_t default_value;
          int32_t cur_value;
      };  // parameters of a control.

      std::vector<std::string> formats;     //list of pixel format supported
      std::vector<std::string> resolutions; //list of resolution supported
      std::vector<std::string> framerates;  //list of framerates supported
      std::vector<queryctrl> ctrl; //list of controls available.
      std::string camera_name;
    public:

// Service client call functions

      int enum_device();

      int select_camera(std::string cam_name,bool shutdown);

      int query_control();

      int v4l2_query_menu(int32_t id,int32_t minimum,int32_t maximum,
                          QStringList &modes);

      int set_control(uint32_t id,int value);

      int get_control(uint32_t id, int *cur_value);

      int enum_format(int type,std::string *pix_fmt,int width,
                      int height,std::string *cur_res,std::string *cur_fps);

      int set_format(std::string format,int width,int height,
                      int numerator, int denominator);

      int get_format(std::string *format,std::string *resolution,
                     std::string *framerate);

      void get_value(int index,std::string *control_name,int *control_type,
        int *control_id,int * min_val,int *max_val,int * cur_val,
        int * step_val,int *def_val);

      void get_control_name(int index,std::string *control_name);

      std::vector<std::string> get_list(int type);

      void clear_controls_list();
  };
}// end of namespace rqt_cam

#endif
