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

#include <rqt_cam/image_view.h>
#include <rqt_cam/ui.h>

namespace rqt_cam {






  // Constructor function of class ImageView
  ImageView::ImageView(): rqt_gui_cpp::Plugin() , widget_(0)
  {
    setObjectName("ImageView");
    prev_cam_name.clear();
    save_img= false;
    image_count=0;
    y16FormatFor20CUG= false;
  }






  /************************************************************************************************************
  *  Name	:	initPlugin.
  *  Parameter1 : qt_gui_cpp::PluginContext& context - context of user interface.
  *  Description	:  This function is called when the plugin starts running,
  *                  This function setup the UI window, updates the topic list and binds singal and slots funtions.
  ************************************************************************************************************/

  void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
  {

    widget_ = new QWidget();
    widget_->setWindowTitle("rqt_cam");
    ui_.setupUi(widget_);
    context.addWidget(widget_);
    updateTopicList();

    ui_.topics_cmb->setCurrentIndex(ui_.topics_cmb->findText(""));

    ui_.refresh_topics_pb->setIcon(QIcon::fromTheme("view-refresh"));
    connect(ui_.refresh_topics_pb, SIGNAL(pressed()), this, SLOT(updateTopicList()));

    ui_.save_as_image_pb->setIcon(QIcon::fromTheme("document-save-as"));
    connect(ui_.save_as_image_pb, SIGNAL(pressed()), this, SLOT(onSaveImagePressed()));

    usr.disable_ui_widgets(&ui_);

    ui_.image_frame->setOuterLayout(ui_.image_layout);

    hide_toolbar_action_ = new QAction(tr("Hide toolbar"), this);
    hide_toolbar_action_->setCheckable(true);

    ui_.image_frame->addAction(hide_toolbar_action_);
    connect(hide_toolbar_action_, SIGNAL(toggled(bool)), this, SLOT(onHideToolbarChanged(bool)));
    usr.update_image_format_cmb(&ui_);
  }






  /************************************************************************************************************
  *  Name	:	updateTopicList.
  *  Description	: This function is called when initialisation and when refresh_topics_pb is pressed.
  *                 This function updates the topics which are published of type "ecam_v4l2/image".
  ************************************************************************************************************/
  void ImageView::updateTopicList()
  {
    QSet<QString> message_types;
    message_types.insert("ecam_v4l2/image");
    // fill combo box
    QList<QString> topics = getTopics(message_types).values();
    if(topics.size()>0){
      disconnect(ui_.topics_cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));
      qSort(topics);
      ui_.topics_cmb->clear();
      for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++){
        QString label(*it);
        ui_.topics_cmb->addItem(label, QVariant(*it));
      }
      QString selected = ui_.topics_cmb->currentText();
      connect(ui_.topics_cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));
      selectTopic(selected);
    }
  }





  /************************************************************************************************************
  *  Name	:	getTopics.
  *  Parameter1 : const QSet<QString>& message_types - (i.e) ecam_v4l2/image
  *  Returns : QSet<QString> - list of topics.
  *  Description	: This function is to get the each topic published.
  ************************************************************************************************************/
  QSet<QString> ImageView::getTopics(const QSet<QString>& message_types)
  {
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);
    QSet<QString> topics;

    for (ros::master::V_TopicInfo::const_iterator cnt = topic_info.begin(); cnt != topic_info.end(); cnt++){
      if (message_types.contains(cnt->datatype.c_str())){
        QString topic = cnt->name.c_str();
        // At the begining of the topic name '/' will there so removing that.
        topic.remove("/");
        ROS_INFO("topic:%s",topic.toStdString().c_str());
        topics.insert(topic);
      }
    }
    return topics;
  }





  /************************************************************************************************************
  *  Name	:	selectTopic.
  *  Parameter1 : const QString& topic- selected topic.
  *  Description	: This function is called when a topic is selected in the topics combo box.
  *                 This funciton updates the topics_cmb in the UI.
  ************************************************************************************************************/
  void ImageView::selectTopic(const QString& topic)
  {
    int index = ui_.topics_cmb->findText(topic);
    ui_.topics_cmb->setCurrentIndex(index);
  }





  /************************************************************************************************************
  *  Name	:	onTopicChanged.
  *  Parameter1 : int index -  selected topic index.
  *  Description	: This function is called when the topic is changed from one topic to another topic
  *   This function subscribes to the topic selected and call the service to publish image for the selected topic.
  *   It also sets the image in the rqt_cam UI.
  ************************************************************************************************************/
  void ImageView::onTopicChanged(int index)
  {
    subscriber_.shutdown();
    ros::NodeHandle node_handle;
    // reset image on topic change
    ui_.image_frame->setImage(QImage());

    disconnect_ui_widgets();

    QStringList parts = ui_.topics_cmb->itemData(index).toString().split(" ");
    QString topic = parts.first();

    if (!topic.isEmpty())
    {
      //Checking whether the camera is See3CAM_20CUG
      if(topic.contains("See3CAM_20CUG")){
        y16FormatFor20CUG=true;
      }else{
        y16FormatFor20CUG=false;
      }

      if(prev_cam_name.length()!=0){
         // if previously another camera is selected, requesting to streamoff that camera.
        if(ctl.select_camera(prev_cam_name,OFF)<0){
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
      if(ctl.select_camera(topic.toStdString(),ON)==SUCCESS){
        // At the begining of the topic name '/' will there so adding that.
        subscriber_ = node_handle.subscribe("/"+topic.toStdString(), 1, &ImageView::callbackImage, this);

        usr.disable_ui_widgets(&ui_);

        if(usr.setup_uvc_settings(&ui_,ctl,&ids)<0){
          ROS_INFO("Failed to setup uvc settings");
        }
        prepare_format(&format,&width,&height,&fps_numerator,&fps_denominator);
        if(usr.setup_format_settings(&ui_,ctl,PIX_FORMAT,format,width,height)<0){
          ROS_INFO("Failed to setup format settings");
        }
        connect_ui_widgets();
      }else{           // if camera is Already streaming in another window
        QMessageBox msgBox;
        msgBox.setText("This Camera is already streaming.");
        msgBox.exec();
        ui_.topics_cmb->setCurrentIndex(ui_.topics_cmb->findText(""));
        usr.disable_ui_widgets(&ui_);
        prev_cam_name= "none";
      }
    }
  }





  /************************************************************************************************************
  *  Name	:	onSliderChange.
  *  Parameter1 : int32_t controlId - Control id of the uvc control.
  *  Parameter2 : int value - Control value which is to be changed.
  *  Description	: This is callback function called when slider value is changed(when slider is down).
  *                 This function is used to change uvc setting of the respected slider.
  ************************************************************************************************************/
  void ImageView::onSliderChange(int32_t controlId,int value)
  {
    int cur_value;
    if(ctl.set_control(controlId,value)==0){
      usr.update_slider_value(value,controlId,&ui_,ctl,ids);
    }else{
      ROS_ERROR("Failed to set control");
    }
  }





  /************************************************************************************************************
  *  Name	:	onCurrentValChanged.
  *  Parameter1 : int32_t controlId - Control id of the uvc control.
  *  Parameter2 : int value - Control value which is to be changed.
  *  Description	: This is callback function called when slider is moved(when slider is dragged).
  *                 This function is used to change the slider value label which is displayed in UI.
  ************************************************************************************************************/
  void ImageView::onCurrentValChanged(int32_t controlId,int value)
  {
    usr.update_slider_value(value,controlId,&ui_,ctl,ids);
  }





  /************************************************************************************************************
  *  Name	:	onCheckBoxChanged.
  *  Parameter1 : int32_t controlId - Control id of the uvc control.
  *  Parameter2 : int value - Control value which is to be changed.
  *  Description	: This is callback function called when CheckBox is pressed.
  *                 This function is used to change the uvc control of the respected CheckBox.
  ************************************************************************************************************/
  void ImageView::onCheckBoxChanged(int32_t controlId ,int state)
  {
    int cur_value,value;
    if(state){
     value = 1;
    }else{
     value = 0;
    }
    if(controlId==ids.ExposureAutoId){
     value=!value;
    }
    ctl.set_control(controlId,value);
    usr.update_checkbox_setting(controlId,value,&ui_,ctl,ids);
  }





  /************************************************************************************************************
  *  Name	:	onFormatChange.
  *  Parameter1 : int index - Index of format combo box.
  *  Description	: This is callback function called when format combo box index is changed.
  *                 This function is used to set the format.
  ************************************************************************************************************/
  void ImageView::onFormatChange(int index)
  {
    QObject::disconnect(resolution_cmb_connection  );
    QObject::disconnect( fps_cmb_connection );
    prepare_format(&format,&width,&height,&fps_numerator,&fps_denominator);
    if(usr.setup_format_settings(&ui_,ctl,RESOLUTION,format,width,height)<0){
     ROS_INFO("Failed to setup format settings");
    }
    prepare_format(&format,&width,&height,&fps_numerator,&fps_denominator);
    if(ctl.set_format(format,width,height,fps_numerator,fps_denominator)<0){
     ROS_INFO("Failed to setup format settings");
    }
    resolution_cmb_connection=connect(ui_.resolution_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),this,&ImageView::onResolutionChange);
    fps_cmb_connection=connect(ui_.fps_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),this,&ImageView::onFpsChange);
  }







  /************************************************************************************************************
  *  Name	:	onResolutionChange.
  *  Parameter1 : int index - Index of resolution combo box.
  *  Description	: This is callback function called when resolution combo box index is changed.
  *                 This function is used to set the resolution.
  ************************************************************************************************************/
  void ImageView::onResolutionChange(int index)
  {
    QObject::disconnect( fps_cmb_connection );
    prepare_format(&format,&width,&height,&fps_numerator,&fps_denominator);
    if(usr.setup_format_settings(&ui_,ctl,FPS,format,width,height)<0){
      ROS_INFO("Failed to setup format settings");
    }
    prepare_format(&format,&width,&height,&fps_numerator,&fps_denominator);
    if(ctl.set_format(format,width,height,fps_numerator,fps_denominator)<0){
      ROS_INFO("Failed to setup format settings");
    }
    fps_cmb_connection=connect(ui_.fps_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),this,&ImageView::onFpsChange);
  }





  /************************************************************************************************************
  *  Name	:	onFpsChange.
  *  Parameter1 : int index - Index of framerate combo box.
  *  Description	: This is callback function called when framerate combo box index is changed.
  *                 This function is used to set the framerate.
  ************************************************************************************************************/
  void ImageView::onFpsChange(int index)
  {
    prepare_format(&format,&width,&height,&fps_numerator,&fps_denominator);
    if(ctl.set_format(format,width,height,fps_numerator,fps_denominator)<0){
      ROS_INFO("Failed to setup format settings");
    }
  }



  /************************************************************************************************************
  *  Name	:	prepare_format.
  *  Parameter1 : std::string *format - pixel format which need to be set.
  *  Parameter2 : int  *width - width of the frame.
  *  Parameter3 : int  *height- height of the frame.
  *  Parameter4 : int  *numerator - numerator of framerate.
  *  Parameter5 : int  *denominator - denominator of framerate.
  *  Description	: This function is used to get the format,resolution and framerate from UI.
  ************************************************************************************************************/
  void ImageView::prepare_format(std::string *format,int *width,int *height,int *numerator, int *denominator)
  {
    char dummy_char;
    double fps;
    *format=ui_.format_cmb->currentText().toStdString();
    QString cur_res = ui_.resolution_cmb->currentText();
    std::istringstream temp1_stream(cur_res.toStdString());
    temp1_stream >>*width >> dummy_char >> *height;
    QString cur_fps = ui_.fps_cmb->currentText();
    std::istringstream temp2_stream(cur_fps.toStdString());
    temp2_stream >>fps;
    if((fps-(int)fps)!=0){
      *numerator=2;
      *denominator=(2*fps);
    }else{
      *numerator=1;
      *denominator=fps;
    }
    image_count=0;
  }





  /************************************************************************************************************
  *  Name	:	onResetPressed.
  *  Description	: This is callback function called when Hardware Reset Button is pressed.
  *                 This function is used to set all uvc controls to default value.
  ************************************************************************************************************/
  void ImageView::onResetPressed()
  {
    usr.reset_uvc_settings(&ui_,ctl);
  }





  /************************************************************************************************************
  *  Name	:	onHideToolbarChanged.
  *  Parameter1 : bool hide - true if hide is pressed.
  *  Description	: This function is called when hide_toolbar_action_ is happened
  *                 This function will hide the toolbar_widget widget.
  ************************************************************************************************************/
  void ImageView::onHideToolbarChanged(bool hide)
  {
    ui_.toolbar_widget->setVisible(!hide);
  }





  /************************************************************************************************************
  *  Name	:	disconnect_ui_widgets.
  *  Description	: This function disconnects all the signals and slots.
  ************************************************************************************************************/
  void ImageView::disconnect_ui_widgets()
  {
    QObject::disconnect(brightnessSlider_connection);QObject::disconnect(backlightSlider_connection);
    QObject::disconnect(gammaSlider_connection);QObject::disconnect(focus_auto_checkbox_connection);
    QObject::disconnect(contrastSlider_connection);QObject::disconnect(exposureSlider_connection);
    QObject::disconnect(hueSlider_connection);QObject::disconnect(exposure_auto_checkbox_connection);
    QObject::disconnect(panSlider_connection);QObject::disconnect(focusSlider_connection);
    QObject::disconnect(sharpnessSlider_connection);QObject::disconnect(format_cmb_connection);
    QObject::disconnect(saturationSlider_connection);QObject::disconnect(focus_abs_Slider_connection);
    QObject::disconnect(whitebalanceSlider_connection);QObject::disconnect(resolution_cmb_connection);
    QObject::disconnect(tiltSlider_connection);QObject::disconnect(gainSlider_connection);
    QObject::disconnect(wb_auto_checkbox_connection);QObject::disconnect(fps_cmb_connection);
    QObject::disconnect(zoomSlider_connection);QObject::disconnect(reset_button_connection);
  }





  /************************************************************************************************************
  *  Name	:	connect_ui_widgets.
  *  Description	: This function connects all the signals and slots.
  ************************************************************************************************************/
  void ImageView::connect_ui_widgets()
  {
    brightnessSlider_connection=connect(ui_.brightnessSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.BrightnessId,std::placeholders::_1));
    contrastSlider_connection=connect(ui_.contrastSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.ContrastId,std::placeholders::_1));
    panSlider_connection=connect(ui_.panSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.PanAbsoluteId,std::placeholders::_1));
    saturationSlider_connection=connect(ui_.saturationSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.SaturationId,std::placeholders::_1));
    tiltSlider_connection=connect(ui_.tiltSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.TiltAbsoluteId,std::placeholders::_1));
    zoomSlider_connection=connect(ui_.zoomSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.ZoomAbsoluteId,std::placeholders::_1));
    backlightSlider_connection=connect(ui_.backlightSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.BacklightCompensationId,std::placeholders::_1));
    exposureSlider_connection=connect(ui_.exposureSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.ExposureAbsoluteId,std::placeholders::_1));
    focusSlider_connection=connect(ui_.focusSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.FocusId,std::placeholders::_1));
    focus_abs_Slider_connection=connect(ui_.focus_abs_Slider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.FocusAbsoluteId,std::placeholders::_1));
    gainSlider_connection=connect(ui_.gainSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.GainId,std::placeholders::_1));
    gammaSlider_connection=connect(ui_.gammaSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.GammaId,std::placeholders::_1));
    hueSlider_connection=connect(ui_.hueSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.HueId,std::placeholders::_1));
    sharpnessSlider_connection=connect(ui_.sharpnessSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.SharpnessId,std::placeholders::_1));
    whitebalanceSlider_connection=connect(ui_.whitebalanceSlider,&QAbstractSlider::valueChanged,std::bind(&ImageView::onSliderChange,this,ids.WhiteBalanceTemperatureId,std::placeholders::_1));

    brightness_val_connection=connect(ui_.brightnessSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.BrightnessId,std::placeholders::_1));
    contrast_val_connection=connect(ui_.contrastSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.ContrastId,std::placeholders::_1));
    pan_val_connection=connect(ui_.panSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.PanAbsoluteId,std::placeholders::_1));
    saturation_val_connection=connect(ui_.saturationSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.SaturationId,std::placeholders::_1));
    tilt_val_connection=connect(ui_.tiltSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.TiltAbsoluteId,std::placeholders::_1));
    zoom_val_connection=connect(ui_.zoomSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.ZoomAbsoluteId,std::placeholders::_1));
    backlight_val_connection=connect(ui_.backlightSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.BacklightCompensationId,std::placeholders::_1));
    exposure_val_connection=connect(ui_.exposureSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.ExposureAbsoluteId,std::placeholders::_1));
    focus_val_connection=connect(ui_.focusSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.FocusId,std::placeholders::_1));
    focus_abs_val_connection=connect(ui_.focus_abs_Slider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.FocusAbsoluteId,std::placeholders::_1));
    gain_val_connection=connect(ui_.gainSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.GainId,std::placeholders::_1));
    gamma_val_connection=connect(ui_.gammaSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.GammaId,std::placeholders::_1));
    hue_val_connection=connect(ui_.hueSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.HueId,std::placeholders::_1));
    sharpness_val_connection=connect(ui_.sharpnessSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.SharpnessId,std::placeholders::_1));
    whitebalance_val_connection=connect(ui_.whitebalanceSlider,&QAbstractSlider::sliderMoved,std::bind(&ImageView::onCurrentValChanged,this,ids.WhiteBalanceTemperatureId,std::placeholders::_1));

    wb_auto_checkbox_connection=connect(ui_.wb_auto_checkbox,&QCheckBox::stateChanged,std::bind(&ImageView::onCheckBoxChanged,this,ids.WhiteBalanceTemperatureAutoId,std::placeholders::_1));
    focus_auto_checkbox_connection=connect(ui_.focus_auto_checkbox,&QCheckBox::stateChanged,std::bind(&ImageView::onCheckBoxChanged,this,ids.FocusAutoId,std::placeholders::_1));
    exposure_auto_checkbox_connection=connect(ui_.exposure_auto_checkbox,&QCheckBox::stateChanged,std::bind(&ImageView::onCheckBoxChanged,this,ids.ExposureAutoId,std::placeholders::_1));
    format_cmb_connection=connect(ui_.format_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),this,&ImageView::onFormatChange);
    resolution_cmb_connection=connect(ui_.resolution_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),this,&ImageView::onResolutionChange);
    fps_cmb_connection=connect(ui_.fps_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),this,&ImageView::onFpsChange);
    reset_button_connection= connect(ui_.reset_button,&QAbstractButton::clicked,this,&ImageView::onResetPressed);

  }





  /************************************************************************************************************
  *  Name	:	onSaveImagePressed.
  *  Description	: This function is called when save_as_image_pb in the UI is pressed
  *                    This function is used to set the save_img flag and pop up image save message.
  ************************************************************************************************************/

  void ImageView::onSaveImagePressed()
  {
    save_img= true;
    QMessageBox save_msg;
    save_msg.setText("The image is saved in the workspace folder");
    save_msg.exec();
  }





  /************************************************************************************************************
  *  Name	:	save_image.
  *  Parameter1 :const unsigned char *buffer - buffer having image data.
  *  Parameter2 : std::string format - pixel format which need to be set.
  *  Parameter3 : int  width - width of the frame.
  *  Parameter4 : int  height- height of the frame.
  *  Description	: This function is called when save_as_image_pb in the UI is pressed
  *                    This function saves the current image which is being streamed.
  ************************************************************************************************************/
  void ImageView::save_image(const unsigned char *buffer,std::string format,int length,int width,int height)
  {
    // take a snapshot before asking for the filename
    QImage img = ui_.image_frame->getImageCopy();
    int fp;
    std::stringstream file_name;
    std::string cam_name=prev_cam_name;
    struct tm *tm;
    time_t t = time(0);
    tm = localtime(&t);
    file_name<<cam_name.c_str()<<"_"<<format.c_str()<<"_"<<width<<"x"<<height<<"_"<<tm->tm_mday<<"_"<<tm->tm_mon + 1<<"_"
             <<tm->tm_year + 1900<<"_"<<tm->tm_hour<<"_"<< tm->tm_min<<"_"<<tm->tm_sec<<"_"<<image_count;
     QString cur_fmt = ui_.image_format_cmb->currentText();
     if(cur_fmt=="raw"){
       file_name<<".raw";
       if (((fp = open(file_name.str().c_str(), O_RDONLY)) < 0)) {
         fp = open(file_name.str().c_str(), O_WRONLY | O_CREAT | O_SYNC, S_IRWXU | S_IRGRP | S_IROTH);
         if(fp < 0) {
           perror("open :");
         }
       }
       size_t ret = write(fp,buffer,length);
       if(ret == length){
         ROS_INFO("Image  %s  saved successfully \n",file_name.str().c_str());
       }else{
         ROS_INFO("Error in saving file --Frame  %s  saved with %zu bytes\n",file_name.str().c_str(),ret);
       }
       close(fp);
    }else{
      file_name<<"."<<cur_fmt.toStdString();
      img.save(QString::fromStdString(file_name.str()));
      ROS_INFO("Image  %s  saved successfully \n",file_name.str().c_str());
    }
    image_count++;
  }






  /************************************************************************************************************
  *  Name	:	ConvertY12toY8.
  *  Parameter1 : uchar * Y12Buff - buffer in which is Y12 data is stored.
  *  Parameter2 : int height - height of the frame.
  *  Parameter3 : int width - width of the frame.
  *  Parameter4 : cv::Mat &Y8Buff - buffer in which is Y8 data is to be stored
  *  Description	: This function is to convert Y12 data to Y8 data.
  ************************************************************************************************************/
  bool ImageView::ConvertY12toY8(uchar * Y12Buff,int height,int width, cv::Mat &Y8Buff)
  {
    int cnt1,cnt2,cnt3 = 0;
    uchar *PixelBuff = NULL;
    PixelBuff = new uchar[height * width];
    for (int cnt1 = 0;cnt1 < height;cnt1++){
      for (int cnt2 = 0;cnt2 < width;cnt2 += 2){
      PixelBuff[cnt1*width + cnt2] = Y12Buff[cnt3];
      PixelBuff[cnt1*width + cnt2 + 1] = Y12Buff[cnt3 + 1];
      cnt3 += 3;
      }
    }
    Y8Buff = cv::Mat(height, width, CV_8UC1, PixelBuff);  // convert to Mat from buffer
    delete(PixelBuff);
    return true;
  }





  /************************************************************************************************************
  *  Name	:	ConvertY16toY8.
  *  Parameter1 : uchar * Y16Buff - buffer in which is Y16 data is stored.
  *  Parameter2 : int height - height of the frame.
  *  Parameter3 : int width - width of the frame.
  *  Parameter4 : cv::Mat &Y8Buff - buffer in which is Y8 data is to be stored
  *  Description	: This function is to convert Y16 data to Y8 data.
  ************************************************************************************************************/
  bool ImageView::ConvertY16toY8(uint16_t * Y16Buff,int height,int width, cv::Mat &Y8Buff)
  {
    uchar *PixelBuff = NULL;
    PixelBuff = new uchar[height * width];
    uchar *ptr = PixelBuff;
    /* Y16 to Y8 conversion */
    if(y16FormatFor20CUG){   // Convertion for See3CAM_20CUG camera.
      for(__u32 cnt=0; cnt<(width * height); cnt++) {
          *ptr++ =(Y16Buff[cnt] * 0.2490234375);
      }
    }else{ // Convertion for other cameras.
      uchar *buff = (uchar *)Y16Buff;
      for(__u32 cnt=0; cnt<(width * height*2); cnt=cnt+2) {
        *ptr++ = (((buff[cnt] & 0xF0) >> 4) | (buff[cnt+1] & 0x0F) << 4);
      }
    }
    Y8Buff = cv::Mat(height, width, CV_8UC1, PixelBuff);  // convert to Mat from buffer
    delete(PixelBuff);
    return true;
  }






  /************************************************************************************************************
  *  Name	:	callbackImage.
  *  Parameter1 : const rqt_cam::image::ConstPtr& msg - Topic's Message publsihed by publisher.
  *  Description	: This callback function is called when the image is published by the publisher on the selected topic.
  *                This function uses Opencv bridge to render the image for streaming.
  ************************************************************************************************************/
  void ImageView::callbackImage(const rqt_cam::image::ConstPtr& msg)
  {
    try{

      if(msg->format=="uyvy"){
        cv::Mat mat_src = cv::Mat(msg->height, msg->width, CV_8UC2,(void*)&msg->data[0]);
        cv::cvtColor(mat_src, conversion_mat_, cv::COLOR_YUV2RGB_UYVY);
      }else if (msg->format=="yuyv") {
        cv::Mat mat_src = cv::Mat(msg->height, msg->width, CV_8UC2,(void*)&msg->data[0]);
        cv::cvtColor(mat_src, conversion_mat_, cv::COLOR_YUV2RGB_YUYV);
      }else if(msg->format=="mjpg"){
        cv::Mat Bgr_data;
        // Checking whether mjpeg header is valid.
        if( ((uint8_t*)&msg->data[0])[0] == 0xFF && ((uint8_t*)&msg->data[0])[1] == 0xD8){
          imdecode(cv::Mat(1, msg->length, CV_8U, (void*)&msg->data[0]),cv::IMREAD_COLOR, &Bgr_data);
          cv::cvtColor(Bgr_data,conversion_mat_, cv::COLOR_BGR2RGB);
        }else{
          return;
        }
      }else if(msg->format=="mono8"){
        cv::Mat gray8;
        cv::Mat(msg->height, msg->width, CV_8UC1, (void*)&msg->data[0]).convertTo(gray8, CV_8U);
        cv::cvtColor(gray8,conversion_mat_, cv::COLOR_GRAY2BGR);

      }else if(msg->format=="mono16"){
        cv::Mat gray8;
        ConvertY16toY8((uint16_t*)&msg->data[0],msg->height,msg->width, gray8);
        cv::cvtColor(gray8,conversion_mat_, cv::COLOR_GRAY2BGR);
      }else if(msg->format=="mono12"){
        cv::Mat gray8;
        ConvertY12toY8((uint8_t*)&msg->data[0],msg->height,msg->width, gray8);
        cv::cvtColor(gray8,conversion_mat_, cv::COLOR_GRAY2BGR);
      }
      // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
      QImage image(conversion_mat_.data,msg->width,msg->height, 3*msg->width, QImage::Format_RGB888);
      ui_.image_frame->setImage(image);
      if(save_img){
        save_img=false;
        save_image(&msg->data[0],msg->format,msg->length,msg->width,msg->height);
      }
    }
    catch(cv::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image to 'rgb8' an exception was thrown (%s)", e.what());
      ui_.image_frame->setImage(QImage());
      return;
    }
  }





  /************************************************************************************************************
  *  Name	:	shutdownPlugin.
  *  Description	: This function is called when Ctrl+c is pressed (i.e) when the plugin is stopped
  *                 This function will tell the publisher to stop publishing image and shutdown the subscriber plugin.
  ************************************************************************************************************/
  void ImageView::shutdownPlugin()
  {
    if(ctl.select_camera(prev_cam_name,OFF)<0){
      ROS_INFO("Select camera failed");
    }
    prev_cam_name.clear();
    ROS_INFO("shutdownPlugin");
    subscriber_.shutdown();
  }





}

PLUGINLIB_EXPORT_CLASS(rqt_cam::ImageView, rqt_gui_cpp::Plugin)
