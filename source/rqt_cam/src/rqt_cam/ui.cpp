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
#include<rqt_cam/ui.h>

namespace rqt_cam{


  // Constructor function of class UserInterface
  UserInterface::UserInterface()
  {
    controls_size=0;
  }




  /*****************************************************************************
  *  Name	:	enumerate_device.
  *  Returns	:
  *  			True	- If a camera is plugged or unplugged
  *       False - if no camera is plugged or unplugged.
  *  Description	:   This function is to enumerate the devices connected.
  ******************************************************************************/
  bool UserInterface::enumerate_device()
  {
      if(srv.enum_device()<0){
        return false;
      }
      return true;
  }





  /*****************************************************************************
  *  Name	:	Choose_device.
  *  Parameter1 : std::string cam_name - Name of the camera.
  *  Parameter2 : bool shutdown - flag which denotes whether to stream on
  *                               or stream off the camera.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is used to streamon or streamoff the camera.
  *****************************************************************************/
  int UserInterface::Choose_device(std::string cam_name,bool shutdown)
  {
    if(srv.select_camera(cam_name,shutdown)<0)
    {
      return FAILURE;
    }
    return SUCCESS;
  }






  /*****************************************************************************
  *  Name	:	setup_uvc_settings.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is used to query all the uvc controls and
  *                   enable the controls in UI.
  *****************************************************************************/
  int UserInterface:: setup_uvc_settings(Ui::ImageViewWidget *ui_)
  {
    int index,cnt,control_type,control_id;
    int min_val,max_val,cur_val,step_val,def_val;
    std::string control_name;
    QStringList modes;
    srv.clear_controls_list();
    sliders.clear();
    checkboxes.clear();
    comboboxes.clear();
    SliderControlIds.clear();
    CheckBoxControlIds.clear();
    ComboBoxControlIds.clear();

    controls_size = srv.query_control();
    if(controls_size == 0){
      return FAILURE;
    }
    QHBoxLayout * ui_reset_button_layout = new QHBoxLayout;
    QPushButton * ui_reset_button = new QPushButton;
    ui_reset_button->setIcon(QIcon::fromTheme("view-refresh"));
    ui_reset_button_layout->addWidget(new QLabel("UI reset"));
    ui_reset_button_layout->addSpacerItem(new QSpacerItem(0,10,
                                              QSizePolicy::Expanding,
                                              QSizePolicy::Expanding));
    ui_reset_button_layout->addWidget(ui_reset_button);
    ui_->verticalLayout_4->addLayout(ui_reset_button_layout);
    QMetaObject::Connection ui_reset_connection=connect(ui_reset_button,
      &QAbstractButton::clicked,
      std::bind(&UserInterface::onUIResetPressed,this,ui_));

    for(index=0;index<controls_size;index++){
      srv.get_value(index,&control_name,&control_type,&control_id,&min_val,
                    &max_val,&cur_val,&step_val,&def_val);
      switch (control_type) {
        case V4L2_CTRL_TYPE_BOOLEAN:
          create_checkbox(ui_,control_name,control_id,cur_val);
          break;
        case V4L2_CTRL_TYPE_INTEGER:
          create_slider(ui_,control_name,control_id,min_val,max_val,cur_val,step_val);
          break;
        case V4L2_CTRL_TYPE_MENU:
        case V4L2_CTRL_TYPE_INTEGER_MENU:
          modes.clear();
          srv.v4l2_query_menu(control_id,min_val,max_val,modes);
          create_combobox(ui_,control_name,control_id,min_val,max_val,cur_val,modes);
          break;
        default:
          break;
      }
    }// end of for()

    QHBoxLayout * Hardware_reset_button_layout = new QHBoxLayout;
    QPushButton * Hardware_reset_button = new QPushButton("Hardware Reset");
    Hardware_reset_button_layout->addWidget(Hardware_reset_button);
    ui_->verticalLayout_4->addLayout(Hardware_reset_button_layout);
    QMetaObject::Connection hardware_reset_connection=connect(Hardware_reset_button,
                                                      &QAbstractButton::clicked,
                                                      this,&UserInterface::onResetPressed);
    return SUCCESS;
  }//end of setup_uvc_settings function








  /*****************************************************************************
  *  Name	:	create_checkbox.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : std::string control_name - Name of the control.
  *  Parameter3 : int control_id - Control id.
  *  Parameter4 : int cur_val - current control value.
  *  Description	:This function is used to checkbox for boolean type controls.
  *****************************************************************************/
  void UserInterface::create_checkbox(Ui::ImageViewWidget *ui_,
                            std::string control_name,int control_id,int cur_val)
  {
    QHBoxLayout * layout = new QHBoxLayout;
    QCheckBox * checkbox = new QCheckBox(QString::fromUtf8(control_name.c_str()));
    checkbox ->setChecked(cur_val);
    QMetaObject::Connection connection=connect(checkbox,&QCheckBox::stateChanged,
                                       std::bind(&UserInterface::onCheckBoxChanged,
                                       this,ui_,control_id,std::placeholders::_1));
    layout->addWidget(checkbox);
    ui_->verticalLayout_4->addLayout(layout);
    checkboxes.push_back(checkbox);
    CheckBoxControlIds.push_back(control_id);
    update_slider(ui_,control_id,checkbox->isChecked());
  }







  /*****************************************************************************
  *  Name	:	create_combobox.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : std::string control_name - Name of the control.
  *  Parameter3 : int control_id - Control id.
  *  Parameter4 : int min_val - minimum control value.
  *  Parameter5 : int max_val - maximum control value.
  *  Parameter6 : int cur_val - current control value.
  *  Parameter7 : QStringList &modes - list containing menu options.
  *  Description	:   This function is used to slider for integer type controls.
  *****************************************************************************/
  void UserInterface::create_combobox(Ui::ImageViewWidget *ui_,
                      std::string control_name,int control_id,int min_val,
                      int max_val,int cur_val,QStringList &modes)
  {
    QHBoxLayout * layout = new QHBoxLayout;
    QLabel *name = new QLabel(QString::fromUtf8(control_name.c_str()));
    layout->addWidget(name);
    QComboBox * menu = new QComboBox;
    menu->insertItems(0,modes);
    QMetaObject::Connection connection=connect(menu,
      static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      std::bind(&UserInterface::onComboboxChanged,this,ui_,control_id,std::placeholders::_1));
    menu->setCurrentIndex(cur_val);
    update_slider(ui_,control_id,menu->currentIndex());
    comboboxes.push_back(menu);
    ComboBoxControlIds.push_back(control_id);
    layout->addWidget(menu);
    ui_->verticalLayout_4->addLayout(layout);

  }







  /*****************************************************************************
  *  Name	:	create_slider.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : std::string control_name - Name of the control.
  *  Parameter3 : int control_id - Control id.
  *  Parameter4 : int min_val - minimum control value.
  *  Parameter5 : int max_val - maximum control value.
  *  Parameter6 : int cur_val - current control value.
  *  Parameter7 : int step_val - step Value of the control.
  *  Description	:   This function is used to slider for integer type controls.
  *****************************************************************************/
  void UserInterface::create_slider(Ui::ImageViewWidget *ui_,
                      std::string control_name,int control_id,int min_val,
                      int max_val,int cur_val,int step_val)
  {
    int index=-1;
    QCheckBox * checkbox;
    QComboBox * combobox;
    QHBoxLayout * name_layout = new QHBoxLayout;

    QLabel *name = new QLabel(QString::fromUtf8(control_name.c_str()));
    name_layout->addWidget(name);

    name_layout->setObjectName(QString::number(control_id)+"name_layout");

    QHBoxLayout * slider_layout = new QHBoxLayout;
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setMaximumWidth(420);
    slider->setMinimum(min_val);
    slider->setMaximum(max_val);
    slider->setSingleStep(step_val);

    sliders.push_back(slider);
    SliderControlIds.push_back(control_id);

    QLabel *cur_value = new QLabel(QString::number(cur_val));
    QMetaObject::Connection connection=connect(slider,&QAbstractSlider::valueChanged,
                                       std::bind(&UserInterface::onSliderChange,this,
                                       ui_,cur_value,control_id,std::placeholders::_1));
    slider->setValue(cur_val);

    slider_layout->addWidget(slider);
    slider_layout->addWidget(cur_value);
    slider_layout->setObjectName(QString::number(control_id)+"slider_layout");

    QHBoxLayout * value_layout = new QHBoxLayout;
    QLabel *min = new QLabel(QString::number(min_val));
    QLabel *max = new QLabel(QString::number(max_val));
    value_layout->addWidget(min);
    value_layout->addSpacerItem(new QSpacerItem(0,10,
                                    QSizePolicy::Expanding,
                                    QSizePolicy::Expanding));
    value_layout->addWidget(max);
    value_layout->setObjectName(QString::number(control_id)+"value_layout");

    switch (control_id) {
      case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
          index = get_widget_index_by_id(V4L2_CID_AUTO_WHITE_BALANCE,CHECKBOX);
          if(index!=FAILURE){
            checkbox = checkboxes[index];
            update_slider(ui_,V4L2_CID_AUTO_WHITE_BALANCE,checkbox->isChecked());
          }
          break;
      case V4L2_CID_FOCUS_ABSOLUTE:
          index = get_widget_index_by_id(V4L2_CID_FOCUS_AUTO,CHECKBOX);
          if(index!=FAILURE){
            checkbox = checkboxes[index];
            update_slider(ui_,V4L2_CID_FOCUS_AUTO,checkbox->isChecked());
          }
          break;
      case V4L2_CID_EXPOSURE_ABSOLUTE:
          index = get_widget_index_by_id(V4L2_CID_EXPOSURE_AUTO,COMBOBOX);
          if(index!=FAILURE){
            combobox = comboboxes[index];
            update_slider(ui_,V4L2_CID_EXPOSURE_AUTO,combobox->currentIndex());
          }
          index = get_widget_index_by_id(V4L2_CID_FRAME_SYNC,COMBOBOX);
          if(index!=FAILURE){
            combobox = comboboxes[index];
            update_slider(ui_,V4L2_CID_FRAME_SYNC,combobox->currentIndex());
          }
          break;
      default:
          break;
    }

    ui_->verticalLayout_4->addLayout(name_layout);
    ui_->verticalLayout_4->addLayout(slider_layout);
    ui_->verticalLayout_4->addLayout(value_layout);

  }







  /*****************************************************************************
  *  Name	:	onCheckBoxChanged.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int32_t controlId - Control id of the uvc control.
  *  Parameter3 : int value - Control value which is to be changed.
  *  Description	: This is callback function called when CheckBox is pressed.
  *                 and to change the uvc control of the respected CheckBox.
  *****************************************************************************/
  void UserInterface::onCheckBoxChanged(Ui::ImageViewWidget *ui_,
                                        int32_t controlId,int value)
  {
    if(srv.set_control(controlId,value)==0){
      update_slider(ui_,controlId,value);
    }else{
      ROS_ERROR("Failed to set control");
    }
  }






  /*****************************************************************************
  *  Name	:	onComboboxChanged.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int32_t controlId - Control id of the uvc control.
  *  Parameter3 : int value - Control value which is to be changed.
  *  Description	: This is callback function called when Combobox is changed.
  *                 and to change the uvc control of the respected Combobox.
  *****************************************************************************/
  void UserInterface::onComboboxChanged(Ui::ImageViewWidget *ui_,
                                        int32_t controlId,int value)
  {
    if(srv.set_control(controlId,value)==0){
      update_slider(ui_,controlId,value);
    }else{
      ROS_ERROR("Failed to set control");
    }
  }





  /*****************************************************************************
  *  Name	:	onSliderChange.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int32_t controlId - Control id of the uvc control.
  *  Parameter3 : int value - Control value which is to be changed.
  *  Description	: This is callback function called when
  *                 slider value is changed(when slider is down).
  *                 and to change uvc setting of the respected slider.
  *****************************************************************************/
  void UserInterface::onSliderChange(Ui::ImageViewWidget *ui_,QLabel *label,
                                     int32_t controlId,int value)
  {
    if(srv.set_control(controlId,value)==0){
      label->setText(QString::number(value));
    }else{
      ROS_ERROR("Failed to set control");
    }
  }





  /*****************************************************************************
  *  Name	:	update_slider.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int32_t controlId - Control id of the uvc control.
  *  Parameter3 : int value - Control value which is to be changed.
  *  Description:This function is called when checkbox or combobox is changed.
  *              and to update the respected slider(whether to enable or disable).
  *****************************************************************************/
  void UserInterface::update_slider(Ui::ImageViewWidget *ui_,
                                    int32_t controlId,int value)
  {
    int index;
    switch (controlId) {
      case V4L2_CID_EXPOSURE_AUTO:
        index=get_widget_index_by_id(V4L2_CID_EXPOSURE_ABSOLUTE,SLIDER);
        if(value>1){value=false;}
        if(index!=FAILURE)
          sliders[index]->setEnabled(value);
        break;
      case V4L2_CID_AUTO_WHITE_BALANCE:
        index = get_widget_index_by_id(V4L2_CID_WHITE_BALANCE_TEMPERATURE,SLIDER);
        if(index!=FAILURE)
          sliders[index]->setEnabled(!value);
        break;
      case V4L2_CID_FOCUS_AUTO:
        index = get_widget_index_by_id(V4L2_CID_FOCUS_ABSOLUTE,SLIDER);
        if(index!=FAILURE)
          sliders[index]->setEnabled(!value);
        break;
      case V4L2_CID_FRAME_SYNC:
        index = get_widget_index_by_id(V4L2_CID_EXPOSURE_ABSOLUTE,SLIDER);
        if(index!=FAILURE)
          sliders[index]->setEnabled(value);
        index = get_widget_index_by_id(V4L2_CID_EXPOSURE_AUTO,COMBOBOX);
        if(index!=FAILURE){
          comboboxes[index]->setEnabled(!value);
          update_slider(ui_,V4L2_CID_EXPOSURE_AUTO,comboboxes[index]->currentIndex());
        }
        break;
    }
  }




  /*****************************************************************************
  *  Name	:	onResetPressed.
  *  Description:This Callback function called when Hardware Reset Button is pressed.
  *              and to set all uvc controls to default value.
  *****************************************************************************/
  void UserInterface::onResetPressed()
  {
    int index,cnt,control_type,control_id,set_value;
    int min_val,max_val,cur_val,step_val,def_val;
    std::string control_name;
    for(int cnt=0;cnt<controls_size;cnt++){
      srv.get_value(cnt,&control_name,&control_type,&control_id,&min_val,&max_val,
                    &cur_val,&step_val,&def_val);
      switch (control_type) {
        case V4L2_CTRL_TYPE_BOOLEAN:
          index = get_widget_index_by_id(control_id,CHECKBOX);
          checkboxes[index]->setChecked(def_val);
          break;
        case V4L2_CTRL_TYPE_MENU:
        case V4L2_CTRL_TYPE_INTEGER_MENU:
          index = get_widget_index_by_id(control_id,COMBOBOX);
          comboboxes[index]->setCurrentIndex(def_val);
          break;
        case V4L2_CTRL_TYPE_INTEGER:
          index = get_widget_index_by_id(control_id,SLIDER);
          sliders[index]->setValue(def_val);
          break;
        default:
          break;
      }
    }// end of for()
  }





  /*****************************************************************************
  *  Name	:	onUIResetPressed.
  *  Description:This is callback function called when Ui Reset Button is pressed.
  *              and to set all uvc controls widgets to current value.
  *****************************************************************************/
  void UserInterface::onUIResetPressed(Ui::ImageViewWidget *ui_)
  {
    int index,cnt,control_type,control_id,set_value;
    int min_val,max_val,cur_val,step_val,def_val;
    std::string control_name;
    std::string format,resolution,framerate;

    for(int cnt=0;cnt<controls_size;cnt++){
      srv.get_value(cnt,&control_name,&control_type,&control_id,&min_val,&max_val,
                    &cur_val,&step_val,&def_val);
      srv.get_control(control_id,&cur_val);
      switch (control_type) {
        case V4L2_CTRL_TYPE_BOOLEAN:
          index = get_widget_index_by_id(control_id,CHECKBOX);
          checkboxes[index]->setChecked(cur_val);
          break;
        case V4L2_CTRL_TYPE_MENU:
        case V4L2_CTRL_TYPE_INTEGER_MENU:
          index = get_widget_index_by_id(control_id,COMBOBOX);
          comboboxes[index]->setCurrentIndex(cur_val);
          break;
        case V4L2_CTRL_TYPE_INTEGER:
          index = get_widget_index_by_id(control_id,SLIDER);
          sliders[index]->setValue(cur_val);
          break;
        default:
          break;
      }
    }
    srv.get_format(&format,&resolution,&framerate);
    QObject::disconnect(format_cmb_connection);
    QObject::disconnect(resolution_cmb_connection);
    QObject::disconnect(fps_cmb_connection);
    ui_->format_cmb->setCurrentIndex(ui_->format_cmb->
                                      findText(QString::fromStdString(format)));
    if(setup_format_settings(ui_,RESOLUTION)<0){
     ROS_INFO("Failed to setup format settings");
    }
    ui_->resolution_cmb->setCurrentIndex(ui_->resolution_cmb->
                                  findText(QString::fromStdString(resolution)));
    ui_->fps_cmb->setCurrentIndex(ui_->fps_cmb->
                                   findText(QString::fromStdString(framerate)));
    connect_ui_widgets(ui_);
  }







  /*****************************************************************************
  *  Name	:	get_widget_index_by_id.
  *  Parameter1 : int control_id - Control id.
  *  Parameter3 : int type - This denotes whether to return slider or checkbox
  *                          or ComboBox index.
                  The possible values are:
                  SLIDER                    1
                  CHECKBOX                  2
                  COMBOBOX                  3
  *  Returns	:	Function returns the index based on the control id and type.
  *  Description	:   This function is used to get the index of widgets
  *                   by their control id and type.
  *****************************************************************************/
  int UserInterface::get_widget_index_by_id(int control_id,int type)
  {
    switch (type) {
      case SLIDER:
        for (int cnt = 0; cnt != SliderControlIds.size(); ++cnt) {
          if (SliderControlIds[cnt] == control_id) {
            return cnt;
          }
        }
        break;
      case CHECKBOX:
        for (int cnt = 0; cnt != CheckBoxControlIds.size(); ++cnt) {
          if (CheckBoxControlIds[cnt] == control_id) {
            return cnt;
          }
        }
        break;
      case COMBOBOX:
        for (int cnt = 0; cnt != ComboBoxControlIds.size(); ++cnt) {
          if (ComboBoxControlIds[cnt] == control_id) {
            return cnt;
          }
        }
        break;
      default:
        break;
    }
    return FAILURE;
  }






  /*****************************************************************************
  *  Name	:	setup_format_settings.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter3 : int type - This denotes whether to enumerate pixelformat
  *                           or resolution or framerate.
                  The possible values are:
                  PIX_FORMAT                    1
                  RESOLUTION                    2
                  FPS                           3
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is used to enumerate format,resolution and
  *                   framerate and add to ComboBox in UI.
  *****************************************************************************/
  int UserInterface:: setup_format_settings(Ui::ImageViewWidget *ui_,int type)
  {
    int index;
    std::vector<std::string> temp_list;
    std::string cur_res,cur_fps;
    prepare_format(ui_,&format,&width,&height,&fps_numerator,&fps_denominator);
    if(srv.enum_format(type,&format,width,height,&cur_res,&cur_fps)<0){
      return FAILURE;
    }

    switch (type) {
      case PIX_FORMAT:
        ui_->format_cmb->clear();
        temp_list=srv.get_list(PIX_FORMAT);
        for(index=0;index<temp_list.size();index++){
          ui_->format_cmb->addItem(QString::fromStdString(temp_list[index]));
        }
          ui_->format_cmb->setCurrentIndex(ui_->format_cmb->
                                      findText(QString::fromStdString(format)));
      case RESOLUTION:
        ui_->resolution_cmb->clear();
        temp_list=srv.get_list(RESOLUTION);
        for(index=0;index<temp_list.size();index++){
          ui_->resolution_cmb->addItem(QString::fromStdString(temp_list[index]));
        }
          ui_->resolution_cmb->setCurrentIndex(ui_->resolution_cmb->
                                     findText(QString::fromStdString(cur_res)));
      case FPS:
        ui_->fps_cmb->clear();
        temp_list=srv.get_list(FPS);
        for(index=0;index<temp_list.size();index++){
          ui_->fps_cmb->addItem(QString::fromStdString(temp_list[index]));
        }
      default:
        break;
    }
    return SUCCESS;
  }






  /*****************************************************************************
  *  Name	:	update_image_format_cmb.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Description	:   This function is used to update the image format ComboBox.
  *****************************************************************************/
  void UserInterface::update_image_format_cmb(Ui::ImageViewWidget *ui_)
  {
    ui_->image_format_cmb->addItem("jpg");
    ui_->image_format_cmb->addItem("bmp");
    ui_->image_format_cmb->addItem("png");
    ui_->image_format_cmb->addItem("raw");

  }







  /*****************************************************************************
  *  Name	:	connect_ui_widgets.
  *  Description	: This function connects the signals and slots
  *                 for image quality settings.
  *****************************************************************************/
  void UserInterface::connect_ui_widgets(Ui::ImageViewWidget *ui_)
  {
    format_cmb_connection=connect(
                          ui_->format_cmb,static_cast<void (QComboBox::*)(int)>
                          (&QComboBox::currentIndexChanged),
                          std::bind(&UserInterface::onFormatChange,
                                    this,ui_,std::placeholders::_1)
                          );
    resolution_cmb_connection=connect(
                              ui_->resolution_cmb,static_cast<void (QComboBox::*)(int)>
                              (&QComboBox::currentIndexChanged),
                              std::bind(&UserInterface::onResolutionChange,this,
                                        ui_,std::placeholders::_1)
                              );
    fps_cmb_connection=connect(
                              ui_->fps_cmb,static_cast<void (QComboBox::*)(int)>
                              (&QComboBox::currentIndexChanged),
                              std::bind(&UserInterface::onFpsChange,this,
                              ui_,std::placeholders::_1)
                              );

  }






  /*****************************************************************************
  *  Name	:	disconnect_ui_widgets.
  *  Description	: This function disconnects all the signals and slots and
                    removes all the widgets in UVC control Settings.
  *****************************************************************************/
  void UserInterface::disconnect_ui_widgets(Ui::ImageViewWidget *ui_)
  {
    QLayoutItem *item1,*item2;
    QLayout *layout1;
    QWidget *widget1;
    while(ui_->verticalLayout_4->count()){
        item1 = ui_->verticalLayout_4->takeAt(0);
        if(item1){
            layout1 = item1->layout();
            if(layout1)
            {
                while(layout1->count()){
                    item2 = layout1->takeAt(0);
                    widget1 = item2->widget();
                    if(widget1){
                      widget1->deleteLater();
                    }
                }
                layout1->deleteLater();
            }
        }
    }

    QObject::disconnect(format_cmb_connection);
    QObject::disconnect(resolution_cmb_connection);
    QObject::disconnect(fps_cmb_connection);
  }






  /*****************************************************************************
  *  Name	:	onFormatChange.
  *  Parameter1 : int index - Index of format combo box.
  *  Description	: This is callback function called when format combo box index
  *                 is changed.This function is used to set the format.
  *****************************************************************************/
  void UserInterface::onFormatChange(Ui::ImageViewWidget *ui_,int index)
  {
    QObject::disconnect(resolution_cmb_connection  );
    QObject::disconnect( fps_cmb_connection );
    if(setup_format_settings(ui_,RESOLUTION)<0){
     ROS_INFO("Failed to setup format settings");
    }

    prepare_format(ui_,&format,&width,&height,&fps_numerator,&fps_denominator);
    if(srv.set_format(format,width,height,fps_numerator,fps_denominator)<0){
     ROS_INFO("Failed to setup format settings");
    }
    resolution_cmb_connection=connect(
                              ui_->resolution_cmb,static_cast<void (QComboBox::*)(int)>
                              (&QComboBox::currentIndexChanged),
                              std::bind(&UserInterface::onResolutionChange,this,
                              ui_,std::placeholders::_1)
                              );
    fps_cmb_connection=connect(
                       ui_->fps_cmb,static_cast<void (QComboBox::*)(int)>
                       (&QComboBox::currentIndexChanged),
                       std::bind(&UserInterface::onFpsChange,this,
                       ui_,std::placeholders::_1)
                       );

  }







  /*****************************************************************************
  *  Name	:	onResolutionChange.
  *  Parameter1 : int index - Index of resolution combo box.
  *  Description	: This is callback function called when resolution combo box
  *                 index is changed.This function is used to set the resolution.
  *****************************************************************************/
  void UserInterface::onResolutionChange(Ui::ImageViewWidget *ui_,int index)
  {
    QObject::disconnect( fps_cmb_connection );
    if(setup_format_settings(ui_,FPS)<0){
      ROS_INFO("Failed to setup format settings");
    }
    prepare_format(ui_,&format,&width,&height,&fps_numerator,&fps_denominator);
    if(srv.set_format(format,width,height,fps_numerator,fps_denominator)<0){
      ROS_INFO("Failed to setup format settings");
    }
    fps_cmb_connection=connect(
                       ui_->fps_cmb,static_cast<void (QComboBox::*)(int)>
                       (&QComboBox::currentIndexChanged),
                       std::bind(&UserInterface::onFpsChange,this,
                       ui_,std::placeholders::_1)
                       );

  }





  /*****************************************************************************
  *  Name	:	onFpsChange.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int index - Index of framerate combo box.
  *  Description	: This is callback function called when framerate combo box
  *                 index is changed.This function is used to set the framerate.
  *****************************************************************************/
  void UserInterface::onFpsChange(Ui::ImageViewWidget *ui_,int index)
  {
    prepare_format(ui_,&format,&width,&height,&fps_numerator,&fps_denominator);
    if(srv.set_format(format,width,height,fps_numerator,fps_denominator)<0){
      ROS_INFO("Failed to setup format settings");
    }
  }






  /*****************************************************************************
  *  Name	:	prepare_format.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : std::string *format - pixel format which need to be set.
  *  Parameter3 : int  *width - width of the frame.
  *  Parameter4 : int  *height- height of the frame.
  *  Parameter5 : int  *numerator - numerator of framerate.
  *  Parameter6 : int  *denominator - denominator of framerate.
  *  Description	: This function is used to get the format,resolution
  *                 and framerate from UI.
  *****************************************************************************/
  void UserInterface::prepare_format(Ui::ImageViewWidget *ui_,std::string *format,
                        int *width,int *height,int *numerator, int *denominator)
  {
    char dummy_char;
    double fps;
    *format=ui_->format_cmb->currentText().toStdString();
    QString cur_res = ui_->resolution_cmb->currentText();
    std::istringstream temp1_stream(cur_res.toStdString());
    temp1_stream >>*width >> dummy_char >> *height;
    QString cur_fps = ui_->fps_cmb->currentText();
    std::istringstream temp2_stream(cur_fps.toStdString());
    temp2_stream >>fps;
    if((fps-(int)fps)!=0){
      *numerator=2;
      *denominator=(2*fps);
    }else{
      *numerator=1;
      *denominator=fps;
    }
  }






}// end of namespace rqt_cam
