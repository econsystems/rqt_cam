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
#ifndef UI_H
#define UI_H

#include <rqt_cam/srv_clients.h>
#include "rqt_cam/query_control.h"
#include "rqt_cam/set_control.h"
#include <ui_image_view.h>
#include <QSlider>
#include <QWidget>
#include <Qt>
#include <QObject>
#include <QLabel>
#include <QList>
#include <QCheckBox>

#include <rqt_gui_cpp/plugin.h>
#define SLIDER     1
#define CHECKBOX   2
#define COMBOBOX   3
#define V4L2_CID_FRAME_SYNC V4L2_CID_CAMERA_CLASS_BASE+42

namespace rqt_cam{

  class UserInterface: public rqt_gui_cpp::Plugin
  {
    Q_OBJECT

    private:
      int controls_size;
      Services srv;         // object of class Services.
      int width,height,fps_numerator,fps_denominator;
      std::string format;

      QMetaObject::Connection format_cmb_connection,
                              resolution_cmb_connection,
                              fps_cmb_connection;

      std::vector<QSlider *> sliders;
      std::vector<QCheckBox *> checkboxes;
      std::vector<QComboBox *> comboboxes;
      std::vector<int> SliderControlIds;
      std::vector<int> CheckBoxControlIds;
      std::vector<int> ComboBoxControlIds;

    public:
      UserInterface();

      int setup_uvc_settings(Ui::ImageViewWidget *ui_);

      int setup_format_settings(Ui::ImageViewWidget *ui_,int type);

      int Choose_device(std::string cam_name,bool shutdown);

      void update_image_format_cmb(Ui::ImageViewWidget *ui_);

      virtual void prepare_format(Ui::ImageViewWidget *ui_,std::string *format,
                      int *width,int *height,int *numerator, int *denominator);

      void create_slider(Ui::ImageViewWidget *ui_,std::string control_name,
          int control_id,int min_val,int max_val,int cur_val,int step_val);

      void create_combobox(Ui::ImageViewWidget *ui_,std::string control_name,
        int control_id,int min_val,int max_val,int cur_val,QStringList &modes);

      void create_checkbox(Ui::ImageViewWidget *ui_,std::string control_name,
                           int control_id,int cur_val);

      void connect_ui_widgets(Ui::ImageViewWidget *ui_);

      void disconnect_ui_widgets(Ui::ImageViewWidget *ui_);

      void update_slider(Ui::ImageViewWidget *ui_,int32_t controlId,int value);

      int get_widget_index_by_id(int control_id,int type);

      bool enumerate_device();

    protected slots:


      virtual void onSliderChange(Ui::ImageViewWidget *ui_,QLabel *label,
                                  int32_t controlId,int value);

      virtual void onComboboxChanged(Ui::ImageViewWidget *ui_,
                                     int32_t controlId,int value);

      virtual void onCheckBoxChanged(Ui::ImageViewWidget *ui_,
                                     int32_t controlId,int value);

      virtual void onFormatChange(Ui::ImageViewWidget *ui_,int index);

      virtual void onResolutionChange(Ui::ImageViewWidget *ui_,int index);

      virtual void onFpsChange(Ui::ImageViewWidget *ui_,int index);

      virtual void onResetPressed();

      virtual void onUIResetPressed(Ui::ImageViewWidget *ui_);
  };// end of class UserInterface

}// end of namespace rqt_cam
#endif // UI_H
