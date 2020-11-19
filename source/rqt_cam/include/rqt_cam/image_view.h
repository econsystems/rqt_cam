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


#ifndef rqt_cam_H
#define rqt_cam_H

#include <rqt_gui_cpp/plugin.h>
#include <fcntl.h>
#include <signal.h>
#include <pluginlib/class_list_macros.h>
#include <ui_image_view.h>
#include <cv_bridge/cv_bridge.h>
#include <QAction>
#include <QImage>
#include <QWidget>
#include <QObject>
#include <QMessageBox>
#include <QMainWindow>
#include <QMouseEvent>
#include <QEvent>
#include <rqt_cam/ui.h>
#include <rqt_cam/conversion.h>
#include "rqt_cam/image.h"
#define ON              false
#define OFF             true
#define BPP_FOR_Y8      1
#define BPP_FOR_RGB     3
enum y16formatcameras
{
  OTHER,
  SEE3CAM_CU40,
  SEE3CAM_20CUG,
};
#define realloc_buffer(h,w,bpp) DestBuffer=(unsigned char*)realloc(DestBuffer,h*w*bpp);
namespace rqt_cam {

  class ImageView : public rqt_gui_cpp::Plugin
  {

    Q_OBJECT

    private:

      QAction* hide_toolbar_action_;
      UserInterface usr;          // object of class UserInterface.
      Conversion cvrt;            // object of class Conversion.
      int image_count;
      int old_index;
      std::string prev_cam_name;  // to store previously selected
      bool save_img;
      y16formatcameras y16Format; /* value will be based on camera.
                                     See y16formatcameras enum*/


    protected:

      Ui::ImageViewWidget ui_;

      QWidget* widget_;

      ros::Subscriber subscriber_;

      cv::Mat input_mat_,output_mat_;

      unsigned char  *DestBuffer;

      virtual void check_camera_name(QString topic);

      virtual QSet<QString> getTopics(const QSet<QString>& message_types);

      virtual void selectTopic(const QString& topic);

      virtual void callbackImage(const rqt_cam::image::ConstPtr& msg);

      bool eventFilter(QObject *obj, QEvent *ev);

    protected slots:

      virtual void updateTopicList();

      virtual void onTopicChanged(int index);

      virtual void onSaveImagePressed();

      virtual void onHideToolbarChanged(bool hide);

      virtual void save_image(const unsigned char *buffer,std::string format,
                              int length,int width,int height);


    public:

      ImageView();

      virtual void initPlugin(qt_gui_cpp::PluginContext& context);

      virtual void shutdownPlugin();

  };// end of class ImageView

}// end of namespace rqt_cam

#endif // RQT_CAM_H
