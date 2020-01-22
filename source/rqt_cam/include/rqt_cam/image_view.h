/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
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
 *   * Neither the name of the TU Darmstadt nor the names of its
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

#ifndef rqt_cam_H
#define rqt_cam_H

#include <rqt_gui_cpp/plugin.h>
#include <fcntl.h>
#include <pluginlib/class_list_macros.h>
#include <ui_image_view.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QAction>
#include <QImage>
#include <QWidget>
#include <QObject>
#include <QMessageBox>
#include <QMainWindow>
#include <rqt_cam/controls.h>
#include <rqt_cam/ui.h>
#include "rqt_cam/image.h"
#define ON false
#define OFF true
namespace rqt_cam {

  class ImageView : public rqt_gui_cpp::Plugin
  {

    Q_OBJECT

    private:

      QAction* hide_toolbar_action_;

      UserInterface usr;    // object of class UserInterface.
      Controls ctl;         // object of class Controls.
      struct controlid ids; // Structure variable of struct controlid.

      int width,height,fps_numerator,fps_denominator;
      int image_count;
      std::string format;
      std::string prev_cam_name; // to store previously selected
      bool save_img;
      bool y16FormatFor20CUG;

      // signal and slot connections
      QMetaObject::Connection brightnessSlider_connection,contrastSlider_connection,panSlider_connection,
      saturationSlider_connection,tiltSlider_connection,zoomSlider_connection,backlightSlider_connection,
      exposureSlider_connection,focusSlider_connection,focus_abs_Slider_connection,gainSlider_connection,
      gammaSlider_connection,hueSlider_connection,sharpnessSlider_connection,whitebalanceSlider_connection,
      wb_auto_checkbox_connection,focus_auto_checkbox_connection,exposure_auto_checkbox_connection,
      format_cmb_connection,resolution_cmb_connection,fps_cmb_connection,brightness_val_connection,
      contrast_val_connection,pan_val_connection,saturation_val_connection,tilt_val_connection,
      zoom_val_connection,backlight_val_connection,exposure_val_connection,focus_val_connection,
      focus_abs_val_connection,gain_val_connection,gamma_val_connection,hue_val_connection,
      sharpness_val_connection,whitebalance_val_connection,reset_button_connection,save_as_raw;

    protected:

      Ui::ImageViewWidget ui_;

      QWidget* widget_;

      ros::Subscriber subscriber_;

      cv::Mat conversion_mat_;

      virtual QSet<QString> getTopics(const QSet<QString>& message_types);

      virtual void selectTopic(const QString& topic);

      virtual void prepare_format(std::string *format,int *width,int *height,int *numerator, int *denominator);

      virtual void disconnect_ui_widgets();

      virtual void connect_ui_widgets();

      virtual void callbackImage(const rqt_cam::image::ConstPtr& msg);

    protected slots:

      virtual void updateTopicList();

      virtual void onTopicChanged(int index);

      virtual void onSaveImagePressed();

      virtual void save_image(const unsigned char *buffer,std::string format,int length,int width,int height);

      virtual void onHideToolbarChanged(bool hide);

      virtual void onSliderChange(int32_t controlId,int value);

      virtual void onCurrentValChanged(int32_t controlId,int value);

      virtual void onCheckBoxChanged(int32_t controlId,int state);

      virtual void onFormatChange(int index);

      virtual void onResolutionChange(int index);

      virtual void onFpsChange(int index);

      virtual void onResetPressed();

      bool ConvertY12toY8(uchar * Y12Buff,int height,int width, cv::Mat &Y8Buff);

      bool ConvertY16toY8(uint16_t * Y16Buff,int height,int width, cv::Mat &YUYVBuff);

    public:

      ImageView();

      virtual void initPlugin(qt_gui_cpp::PluginContext& context);

      virtual void shutdownPlugin();

  };// end of class ImageView

}// end of namespace rqt_cam

#endif // RQT_CAM_H
