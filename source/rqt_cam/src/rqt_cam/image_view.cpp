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

#include <rqt_cam/image_view.h>
#include <rqt_cam/ui.h>

namespace rqt_cam {






  // Constructor function of class ImageView
  ImageView::ImageView(): rqt_gui_cpp::Plugin() , widget_(0)
  {
    setObjectName("ImageView");
    prev_cam_name.clear();
    save_img = false;
    image_count = 0;
    y16Format = OTHER;
    DestBuffer = NULL;
  }






  /*****************************************************************************
  *  Name	:	initPlugin.
  *  Parameter1 : qt_gui_cpp::PluginContext& context - context of user interface.
  *  Description	:  This function is called when the plugin starts running,
  *                  This function setup the UI window, updates the topic list
                     and binds singal and slots funtions.
  *****************************************************************************/

  void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
  {

    widget_ = new QWidget();
    widget_->setWindowTitle("rqt_cam");
    ui_.setupUi(widget_);
    context.addWidget(widget_);
    updateTopicList();

    ui_.topics_cmb->setCurrentIndex(ui_.topics_cmb->findText("Select Device"));
    ui_.topics_cmb->installEventFilter(this);
    ui_.settings_tab->setEnabled(false);

    ui_.save_as_image_pb->setIcon(QIcon::fromTheme("document-save-as"));
    connect(ui_.save_as_image_pb, SIGNAL(pressed()), this,
                                  SLOT(onSaveImagePressed()));
    ui_.save_as_image_pb->setEnabled(false);

    ui_.image_frame->setOuterLayout(ui_.image_layout);

    hide_toolbar_action_ = new QAction(tr("Hide toolbar"), this);
    hide_toolbar_action_->setCheckable(true);

    ui_.image_frame->addAction(hide_toolbar_action_);
    connect(hide_toolbar_action_, SIGNAL(toggled(bool)), this,
                                  SLOT(onHideToolbarChanged(bool)));
    usr.update_image_format_cmb(&ui_);

  }


  /*****************************************************************************
  *  Name	:	eventFilter.
  *  Parameter1 : QObject *obj - topics ComboBox object.
  *  Parameter1 : QEvent *event - Event created by the topics ComboBox.
  *  Description	:  This function is a callback function which is called
                     when an event is created by the topics_cmb
  *****************************************************************************/
  bool ImageView::eventFilter(QObject *obj, QEvent *event)
  {
    if(event->type()==QEvent::MouseButtonPress||event->type()==QEvent::KeyPress)
    {
      if(!subscriber_.getNumPublishers()){
        subscriber_.shutdown();
      }
      usr.enumerate_device();
      updateTopicList();
    }
  }




  /*****************************************************************************
  *  Name	:	updateTopicList.
  *  Description	: This function is called when initialisation and when
                    refresh_topics_pb is pressed.This function updates
                    the topics which are published of type "ecam_v4l2/image".
  *****************************************************************************/
  void ImageView::updateTopicList()
  {
    QSet<QString> message_types;
    message_types.insert("ecam_v4l2/image");
    // fill combo box
    QList<QString> topics = getTopics(message_types).values();
    QString selected = ui_.topics_cmb->currentText();
    disconnect(ui_.topics_cmb,SIGNAL(currentIndexChanged(int)), this,
                              SLOT(onTopicChanged(int)));
    if(topics.size()>0){
      ui_.topics_cmb->clear();
      ui_.topics_cmb->addItem("Select Device");
      qSort(topics);
      for(QList<QString>::const_iterator it = topics.begin();
                                         it != topics.end();
                                         it++)
      {
        QString label(*it);
        ui_.topics_cmb->addItem(label, QVariant(*it));
      }
      selectTopic(selected);
    }else{
      ui_.topics_cmb->clear();
      ui_.topics_cmb->addItem("Select Device");
      prev_cam_name.clear();
      usr.disconnect_ui_widgets(&ui_);
      ui_.settings_tab->setEnabled(false);
    }
  }





  /*****************************************************************************
  *  Name	:	getTopics.
  *  Parameter1 : const QSet<QString>& message_types - (i.e) ecam_v4l2/image
  *  Returns : QSet<QString> - list of topics.
  *  Description	: This function is to get the each topic published.
  *****************************************************************************/
  QSet<QString> ImageView::getTopics(const QSet<QString>& message_types)
  {
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    QSet<QString> topics;

    for (ros::master::V_TopicInfo::const_iterator cnt = topic_info.begin();
                                                  cnt != topic_info.end();
                                                  cnt++)
    {
      if (message_types.contains(cnt->datatype.c_str())){
        QString topic = cnt->name.c_str();
        // At the begining of the topic name '/' will there so removing that.
        topic.remove("/");
        topics.insert(topic);
      }
    }
    return topics;
  }





  /*****************************************************************************
  *  Name	:	selectTopic.
  *  Parameter1 : const QString& topic- selected topic.
  *  Description	: This function is called when a topic is selected
  *                 in the topics combo box.
  *                 This funciton updates the topics_cmb in the UI.
  *****************************************************************************/
  void ImageView::selectTopic(const QString& topic)
  {
    int index = ui_.topics_cmb->findText(topic);
    ui_.topics_cmb->setCurrentIndex(index);
    connect(ui_.topics_cmb, SIGNAL(currentIndexChanged(int)), this,
                            SLOT(onTopicChanged(int)));
  }




  /*****************************************************************************
  *  Name	:	check_camera_name.
  *  Parameter1 :  QString topic- selected topic.
  *  Description	: This function is to check the camera name and
  *                 enable flags accordingly.
  *****************************************************************************/
  void ImageView::check_camera_name(QString topic)
  {
    //Checking whether the camera is See3CAM_20CUG
    if(topic.contains("See3CAM_20CUG")){
      y16Format=SEE3CAM_20CUG;
      return;
    }else if (topic.contains("See3CAM_CU135M_H01R1")){
      y16Format=SEE3CAM_20CUG;
      return;
    }else if (topic.contains("See3CAM_CU135M_H03R1")){
      y16Format=SEE3CAM_20CUG;
      return;
    }else if (topic.contains("See3CAM_CU40")){
      y16Format=SEE3CAM_CU40;
      return;
    }
    else{
      y16Format=OTHER;
    }

    if(DestBuffer){
      free(DestBuffer);
      DestBuffer= NULL;
    }
  }





  /*****************************************************************************
  *  Name	:	onTopicChanged.
  *  Parameter1 : int index -  selected topic index.
  *  Description	: This function is called when the topic is changed from
  *                 one topic to another topic
  *   This function subscribes to the topic selected and call the service to
  *   publish image for the selected topic.
  *   It also sets the image in the rqt_cam UI.
  *****************************************************************************/
  void ImageView::onTopicChanged(int index)
  {
    // if "select device" is selected, it is restricted
    if(!index){
      disconnect(ui_.topics_cmb,SIGNAL(currentIndexChanged(int)), this,
                                SLOT(onTopicChanged(int)));
      ui_.topics_cmb->setCurrentIndex(old_index);
      connect(ui_.topics_cmb,SIGNAL(currentIndexChanged(int)), this,
                             SLOT(onTopicChanged(int)));
      return;
    }

    old_index = index;
    subscriber_.shutdown();
    ros::NodeHandle node_handle;
    // reset image on topic change
    ui_.image_frame->setImage(QImage());
    ui_.settings_tab->setEnabled(true);
    ui_.save_as_image_pb->setEnabled(true);
    usr.disconnect_ui_widgets(&ui_);
    QStringList parts = ui_.topics_cmb->itemData(index).toString().split(" ");
    QString topic = parts.first();

    if (!topic.isEmpty())
    {
      check_camera_name(topic);
      if(prev_cam_name.length()!=0){
         /* if previously another camera is selected,
         requesting to streamoff that camera.*/
        if(usr.Choose_device(prev_cam_name,OFF)<0){
          ROS_INFO("Select camera failed");
        }
        prev_cam_name.clear();
        prev_cam_name = topic.toStdString();
      }else{
        // if previously no camera is selected, it is saved as prev_cam_name
        prev_cam_name.clear();
        prev_cam_name = topic.toStdString();
      }
      // Service call to streamon the camera
      usr.Choose_device(topic.toStdString(),ON);

      // At the begining of the topic name '/' will there so adding that.
      subscriber_ = node_handle.subscribe("/"+topic.toStdString(), 1,
                                          &ImageView::callbackImage, this);
      if(usr.setup_uvc_settings(&ui_)<0){
        ROS_INFO("Failed to setup uvc settings");
      }
      if(usr.setup_format_settings(&ui_,PIX_FORMAT)<0){
        ROS_INFO("Failed to setup format settings");
      }
      usr.connect_ui_widgets(&ui_);
    }
  }




  /*****************************************************************************
  *  Name	:	onHideToolbarChanged.
  *  Parameter1 : bool hide - true if hide is pressed.
  *  Description	: This function is called when hide_toolbar_action is happened
  *                 This function will hide the toolbar_widget widget.
  *****************************************************************************/
  void ImageView::onHideToolbarChanged(bool hide)
  {
    ui_.toolbar_widget->setVisible(!hide);
  }


  /*****************************************************************************
  *  Name	:	onSaveImagePressed.
  *  Description	: This function is called when save_as_image_pb is pressed
  *                 This function is used to set the save_img flag and
  *                 pop up image save message.
  *****************************************************************************/

  void ImageView::onSaveImagePressed()
  {
    save_img= true;
    QMessageBox save_msg;
    save_msg.setText("The image is saved in the workspace folder");
    save_msg.exec();
  }





  /*****************************************************************************
  *  Name	:	save_image.
  *  Parameter1 :const unsigned char *buffer - buffer having image data.
  *  Parameter2 : std::string format - pixel format which need to be set.
  *  Parameter3 : int  width - width of the frame.
  *  Parameter4 : int  height- height of the frame.
  *  Description	: This function is called when save_as_image_pb is pressed
  *                 This function saves the current image which is being streamed.
  *****************************************************************************/
  void ImageView::save_image(const unsigned char *buffer,std::string format,
                              int length,int width,int height)
  {
    // take a snapshot before asking for the filename
    QImage img = ui_.image_frame->getImageCopy();
    int fp;
    std::stringstream file_name;
    std::string cam_name=prev_cam_name;
    struct tm *tm;
    time_t t = time(0);
    tm = localtime(&t);
    file_name<<cam_name.c_str()<<"_"<<format.c_str()<<"_"<<width<<"x"<<height
             <<"_"<<tm->tm_mday<<"_"<<tm->tm_mon + 1<<"_"<<tm->tm_year + 1900
             <<"_"<<tm->tm_hour<<"_"<<tm->tm_min<<"_"<<tm->tm_sec<<"_"<<image_count;
     QString cur_fmt = ui_.image_format_cmb->currentText();
     if(cur_fmt=="raw"){
       file_name<<".raw";
       if (((fp = open(file_name.str().c_str(), O_RDONLY)) < 0)) {
         fp = open(file_name.str().c_str(),O_WRONLY|O_CREAT|O_SYNC,S_IRWXU|S_IRGRP|S_IROTH);
         if(fp < 0) {
           perror("open :");
         }
       }
       size_t ret = write(fp,buffer,length);
       if(ret == length){
         ROS_INFO("Image  %s  saved successfully \n",file_name.str().c_str());
       }else{
         ROS_INFO("Error in saving file --Frame  %s  saved with %zu bytes\n",
                    file_name.str().c_str(),ret);
       }
       close(fp);
    }else{
      file_name<<"."<<cur_fmt.toStdString();
      img.save(QString::fromStdString(file_name.str()));
      ROS_INFO("Image  %s  saved successfully \n",file_name.str().c_str());
    }
    image_count++;
  }







  /*****************************************************************************
  *  Name	:	callbackImage.
  *  Parameter1:const rqt_cam::image::ConstPtr& msg - Topic's Message publsihed
  *  Description : This callback function is called when the image is published
  *                by the publisher on the selected topic.This function uses
  *                Opencv bridge to render the image for streaming.
  *****************************************************************************/
  void ImageView::callbackImage(const rqt_cam::image::ConstPtr& msg)
  {
    try{

      if(msg->format=="uyvy"){

        input_mat_ = cv::Mat(msg->height, msg->width, CV_8UC2,(void*)&msg->data[0]);
        cv::cvtColor(input_mat_, output_mat_, cv::COLOR_YUV2RGB_UYVY);


      }else if(msg->format=="yuyv") {

        input_mat_ = cv::Mat(msg->height, msg->width, CV_8UC2,(void*)&msg->data[0]);
        cv::cvtColor(input_mat_, output_mat_, cv::COLOR_YUV2RGB_YUYV);


      }else if(msg->format=="mjpg"){

        // Checking whether the mjpg frame received is valid frame or not.
        if(((uint8_t*)&msg->data[0])[0]==0xFF&&((uint8_t*)&msg->data[0])[1]==0xD8){
            // converting MJPG to BGR
          imdecode(cv::Mat(1, msg->length, CV_8U, (void*)&msg->data[0]),
                   cv::IMREAD_COLOR,
                   &input_mat_);
          cv::cvtColor(input_mat_,output_mat_, cv::COLOR_BGR2RGB);

        }else{
          return;
        }

      }else if(msg->format=="mono8"){

        cv::Mat(msg->height, msg->width, CV_8UC1,
                (void*)&msg->data[0]).convertTo(input_mat_, CV_8U);
        cv::cvtColor(input_mat_,output_mat_, cv::COLOR_GRAY2BGR);

      }else if(msg->format=="mono16"){
        // if the camera is See3CAM_CU40.
        if(y16Format==SEE3CAM_CU40){

          realloc_buffer(msg->width,msg->height,BPP_FOR_RGB);
          cvrt.ConvertY16toRGB((void*)&msg->data[0],msg->height,msg->width,DestBuffer);
          output_mat_= cv::Mat(msg->height,msg->width, CV_8UC3,DestBuffer);

        // If the camera is See3CAM_20CUG
        }else if(y16Format==SEE3CAM_20CUG){

          realloc_buffer(msg->width,msg->height,BPP_FOR_Y8);
          cvrt.ConvertY16toY8for20CUG((uint16_t*)&msg->data[0],msg->height,msg->width,DestBuffer);
          // convert to Mat from buffer
          input_mat_ = cv::Mat(msg->height,msg->width, CV_8UC1, DestBuffer);
          cv::cvtColor(input_mat_,output_mat_, cv::COLOR_GRAY2BGR);


        }else{

          realloc_buffer(msg->width,msg->height,BPP_FOR_Y8);
          cvrt.ConvertY16toY8((uchar*)&msg->data[0],msg->height,msg->width,DestBuffer);
          // convert to Mat from buffer
          input_mat_ = cv::Mat(msg->height,msg->width, CV_8UC1, DestBuffer);
          cv::cvtColor(input_mat_,output_mat_, cv::COLOR_GRAY2BGR);

        }


      }else if(msg->format=="mono12"){

        realloc_buffer(msg->width,msg->height,BPP_FOR_Y8);
        cvrt.ConvertY12toY8((uint8_t*)&msg->data[0],msg->height,msg->width, DestBuffer);
        // convert to Mat from buffer
        input_mat_ = cv::Mat(msg->height, msg->width, CV_8UC1, DestBuffer);
        cv::cvtColor(input_mat_,output_mat_, cv::COLOR_GRAY2BGR);


      }else if(msg->format=="ba81"){

        realloc_buffer(msg->width,msg->height,BPP_FOR_RGB);
        cvrt.ConvertBY8toRGB((uint8_t *)&msg->data[0], msg->width, msg->height, DestBuffer);
        output_mat_= cv::Mat(msg->height,msg->width, CV_8UC3,DestBuffer);

      }else {  // if the device is unplugged.
        updateTopicList();
      }
      // image must be copied since it uses the conversion_mat_ for storage which is
      // asynchronously overwritten in the next callback invocation
      QImage image(output_mat_.data,
                   output_mat_.cols,
                   output_mat_.rows,
                   output_mat_.step[0],
                   QImage::Format_RGB888);

      ui_.image_frame->setImage(image);
      if(save_img){
        save_img=false;
        save_image(&msg->data[0],msg->format,msg->length,msg->width,msg->height);
      }
    }
    catch(cv::Exception& e)
    {
      qWarning("ImageView.callback_image() throwed an execption (%s)", e.what());
      ui_.image_frame->setImage(QImage());
      return;
    }
  }





  /*****************************************************************************
  *  Name	:	shutdownPlugin.
  *  Description	: This function is called when Ctrl+c is pressed (i.e) when
  *                 the plugin is stopped.This function will tell the publisher
  *                 to stop publishing image and shutdown the subscriber plugin.
  *****************************************************************************/
  void ImageView::shutdownPlugin()
  {
    if(usr.Choose_device(prev_cam_name,OFF)<0){
      ROS_INFO("Select camera failed");
    }
    prev_cam_name.clear();
    ROS_INFO("shutdownPlugin");
    if(DestBuffer){
      free(DestBuffer);
      DestBuffer= NULL;
    }
    subscriber_.shutdown();
  }





}

PLUGINLIB_EXPORT_CLASS(rqt_cam::ImageView, rqt_gui_cpp::Plugin)
