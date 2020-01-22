#include<rqt_cam/ui.h>

namespace rqt_cam{



  UserInterface::UserInterface()
  {
    controls_size=0;
  }

  /************************************************************************************************************
  *  Name	:	setup_uvc_settings.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : Controls &ctl - object of class Controls.
  *  Parameter3 : struct controlid *ids - structure pointer of struct controlid.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is used to query all the uvc controls and enable the controls in UI.
  ************************************************************************************************************/
  int UserInterface:: setup_uvc_settings(Ui::ImageViewWidget *ui_,Controls &ctl,struct controlid *ids)
  {
    int index,cnt;
    int min_val,max_val,cur_val,step_val;
    std::string control_name;
    ctl.clear_controls_list();
    controls_size = ctl.query_control();
    if(controls_size == 0){
      return FAILURE;
    }

    ui_->reset_button->setEnabled(true);
    for(index=0;index<controls_size;index++){

      min_val = ctl.get_value(index,MIN_VAL);
      max_val = ctl.get_value(index,MAX_VAL);
      cur_val = ctl.get_value(index,CUR_VAL);
      step_val = ctl.get_value(index,STEP_VAL);
      ctl.get_control_name(index,&control_name);

      if(control_name.compare("Brightness")==0 ){
        brightnessUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->BrightnessId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Contrast")==0){
        contrastUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->ContrastId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Saturation")==0){
        saturationUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->SaturationId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Pan (Absolute)")==0 || control_name.compare("Pan, Absolute")==0 ){
        panUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->PanAbsoluteId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Tilt (Absolute)")==0 || control_name.compare("Tilt, Absolute")==0 ){
        tiltUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->TiltAbsoluteId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Zoom, Absolute")==0 ){
        zoomUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->ZoomAbsoluteId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Hue")==0){
        hueUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->HueId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("White Balance Temperature")==0){
        whiteBalanceUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->WhiteBalanceTemperatureId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Gamma")==0){
        gammaUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->GammaId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Gain")==0){
        gainUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->GainId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Sharpness")==0){
        sharpnessUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->SharpnessId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Exposure (Absolute)")==0 || control_name.compare("Exposure Time, Absolute")==0 ){
        exposureAbsoluteUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->ExposureAbsoluteId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Focus (absolute)")==0){
        focusAbsoluteUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->FocusAbsoluteId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Backlight Compensation")==0){
        backLightUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->BacklightCompensationId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Focus")==0){
        focusUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->FocusId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("White Balance Temperature, Auto")==0 ||control_name.compare("White Balance, Automatic" )==0){
        whiteBalAutoUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->WhiteBalanceTemperatureAutoId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Focus, Auto")==0){
        autoFocusUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->FocusAutoId = ctl.get_value(index,CTRL_ID);
      }
      else if(control_name.compare("Exposure, Auto")==0 ||control_name.compare("Exposure Auto")==0 ){
        exposureAutoUIupdate(ui_,min_val,max_val,cur_val,step_val);
        ids->ExposureAutoId = ctl.get_value(index,CTRL_ID);
      }
    }// end of for()
    return SUCCESS;
  }//end of setup_uvc_settings function





  /************************************************************************************************************
  *  Name	:	reset_uvc_settings.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : Controls &ctl - object of class Controls.
  *  Description	:   This function is used to reset the uvc controls to default_value.
  ************************************************************************************************************/
  void UserInterface:: reset_uvc_settings(Ui::ImageViewWidget *ui_,Controls &ctl)
  {
    int index,cnt;
    std::string control_name;

    for(index=0;index<controls_size;index++)
    {
      ctl.get_control_name(index,&control_name);
      if(control_name.compare("Brightness")==0 ){
        ui_->brightnessSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Contrast")==0){
        ui_->contrastSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Saturation")==0){
        ui_->saturationSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Pan (Absolute)")==0 || control_name.compare("Pan, Absolute")==0 ){
        ui_->panSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Tilt (Absolute)")==0 || control_name.compare("Tilt, Absolute")==0 ){
        ui_->tiltSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Zoom, Absolute")==0){
        ui_->zoomSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Hue")==0){
        ui_->hueSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("White Balance Temperature")==0){
        if(!ui_->wb_auto_checkbox->isEnabled()){
          ui_->whitebalanceSlider->setEnabled(true);
          ui_->whitebalanceSlider->setTracking(false);
          ui_->whitebalanceValue->setEnabled(true);
          ui_->whitebalancelabel->setEnabled(true);
        }
        ui_->whitebalanceSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Gamma")==0){
        ui_->gammaSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Gain")==0){
        ui_->gainSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Sharpness")==0){
        ui_->sharpnessSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Exposure (Absolute)")==0 || control_name.compare("Exposure Time, Absolute")==0 ){
        if(!ui_->exposure_auto_checkbox->isEnabled()){
          ui_->exposureSlider->setEnabled(true);
          ui_->exposureSlider->setTracking(false);
          ui_->exposureValue->setEnabled(true);
          ui_->exposurelabel->setEnabled(true);
        }
        ui_->exposureSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Focus (absolute)")==0){
        if(!ui_->focus_auto_checkbox->isEnabled()){
          ui_->focus_abs_Slider->setEnabled(true);
          ui_->focus_abs_Slider->setTracking(false);
          ui_->focus_abs_Value->setEnabled(true);
          ui_->focus_abs_label->setEnabled(true);
        }
        ui_->focus_abs_Slider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Backlight Compensation")==0){
        ui_->backlightSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("Focus")==0){
        ui_->focusSlider->setValue(ctl.get_value(index,DEF_VAL));
      }
      else if(control_name.compare("White Balance Temperature, Auto")==0 ||control_name.compare("White Balance, Automatic" )==0){
        if(ctl.get_value(index,DEF_VAL)){
          ui_->wb_auto_checkbox->setChecked(true);
          ui_->whitebalanceSlider->setEnabled(false);
          ui_->whitebalanceValue->setEnabled(false);
          ui_->whitebalancelabel->setEnabled(false);
        }else{
          ui_->wb_auto_checkbox->setChecked(false);
          ui_->whitebalanceSlider->setEnabled(true);
          ui_->whitebalanceSlider->setTracking(false);
          ui_->whitebalanceValue->setEnabled(true);
          ui_->whitebalancelabel->setEnabled(true);
        }
      }
      else if(control_name.compare("Focus, Auto")==0){

        if(ctl.get_value(index,DEF_VAL)){
          ui_->focus_auto_checkbox->setChecked(true);
          ui_->focus_abs_Slider->setEnabled(false);
          ui_->focus_abs_Value->setEnabled(false);
          ui_->focus_abs_label->setEnabled(false);
        }else{
          ui_->focus_auto_checkbox->setChecked(false);
          ui_->focus_abs_Slider->setEnabled(true);
          ui_->focus_abs_Slider->setTracking(false);
          ui_->focus_abs_Value->setEnabled(true);
          ui_->focus_abs_label->setEnabled(true);
        }
      }
      else if(control_name.compare("Exposure, Auto")==0 ||control_name.compare("Exposure Auto")==0 ){
        if(ctl.get_value(index,DEF_VAL)){
          ui_->exposure_auto_checkbox->setChecked(false);
          ui_->exposureSlider->setEnabled(true);
          ui_->exposureSlider->setTracking(false);
          ui_->exposureValue->setEnabled(true);
          ui_->exposurelabel->setEnabled(true);
        }else{
          ui_->exposure_auto_checkbox->setChecked(true);
          ui_->exposureSlider->setEnabled(false);
          ui_->exposureValue->setEnabled(false);
          ui_->exposurelabel->setEnabled(false);
        }
      }

    }// end of for()

  }// end of reset_uvc_settings function






  /************************************************************************************************************
  *  Name	:	brightnessUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of brightness control.
  *  Parameter3 : int max_val - maximum value of brightness control.
  *  Parameter4 : int cur_val - current value of brightness control.
  *  Parameter5 : int step_val - step value of brightness control.
  *  Description	:   This function is used to update the brightness UI widgets.
  ************************************************************************************************************/
  void UserInterface::brightnessUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->brightnessSlider->setEnabled(true);
      ui_->brightnessSlider->setTracking(false);
      ui_->brightnessValue->setEnabled(true);
      ui_->brightnesslabel->setEnabled(true);
      ui_->brightnessSlider->setMinimum(min_val);
      ui_->brightness_min->setNum(min_val);
      ui_->brightnessSlider->setMaximum(max_val);
      ui_->brightness_max->setNum(max_val);
      ui_->brightnessSlider->setValue(cur_val);
      ui_->brightnessSlider->setSingleStep(step_val);
      ui_->brightnessValue->setNum(cur_val);
  }





  /************************************************************************************************************
  *  Name	:	contrastUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of contrast control.
  *  Parameter3 : int max_val - maximum value of contrast control.
  *  Parameter4 : int cur_val - current value of contrast control.
  *  Parameter5 : int step_val - step value of contrast control.
  *  Description	:   This function is used to update the contrast UI widgets.
  ************************************************************************************************************/
  void UserInterface::contrastUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->contrastSlider->setEnabled(true);
      ui_->contrastSlider->setTracking(false);
      ui_->contrastValue->setEnabled(true);
      ui_->contrastlabel->setEnabled(true);
      ui_->contrastSlider->setMinimum(min_val);
      ui_->contrastSlider->setMaximum(max_val);
      ui_->contrastSlider->setValue(cur_val);
      ui_->contrastSlider->setSingleStep(step_val);
      ui_->contrastValue->setNum(cur_val);
      ui_->contrast_min->setNum(min_val);
      ui_->contrast_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	saturationUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of Saturation control.
  *  Parameter3 : int max_val - maximum value of Saturation control.
  *  Parameter4 : int cur_val - current value of Saturation control.
  *  Parameter5 : int step_val - step value of Saturation control.
  *  Description	:   This function is used to update the Saturation UI widgets.
  ************************************************************************************************************/
  void UserInterface::saturationUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->saturationSlider->setEnabled(true);
      ui_->saturationSlider->setTracking(false);
      ui_->saturationValue->setEnabled(true);
      ui_->saturationlabel->setEnabled(true);
      ui_->saturationSlider->setMinimum(min_val);
      ui_->saturationSlider->setMaximum(max_val);
      ui_->saturationSlider->setValue(cur_val);
      ui_->saturationSlider->setSingleStep(step_val);
      ui_->saturationValue->setNum(cur_val);
      ui_->saturation_min->setNum(min_val);
      ui_->saturation_max->setNum(max_val);
  }
  /************************************************************************************************************
  *  Name	:	panUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of pan control.
  *  Parameter3 : int max_val - maximum value of pan control.
  *  Parameter4 : int cur_val - current value of pan control.
  *  Parameter5 : int step_val - step value of pan control.
  *  Description	:   This function is used to update the pan UI widgets.
  ************************************************************************************************************/
  void UserInterface::panUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->panSlider->setEnabled(true);
      ui_->panSlider->setTracking(false);
      ui_->panValue->setEnabled(true);
      ui_->panlabel->setEnabled(true);
      ui_->panSlider->setMinimum(min_val);
      ui_->panSlider->setMaximum(max_val);
      ui_->panSlider->setValue(cur_val);
      ui_->panSlider->setSingleStep(step_val);
      ui_->panValue->setNum(cur_val);
      ui_->pan_min->setNum(min_val);
      ui_->pan_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	tiltUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of tilt control.
  *  Parameter3 : int max_val - maximum value of tilt control.
  *  Parameter4 : int cur_val - current value of tilt control.
  *  Parameter5 : int step_val - step value of tilt control.
  *  Description	:   This function is used to update the tilt UI widgets.
  ************************************************************************************************************/
  void UserInterface::tiltUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->tiltSlider->setEnabled(true);
      ui_->tiltSlider->setTracking(false);
      ui_->tiltValue->setEnabled(true);
      ui_->tiltlabel->setEnabled(true);
      ui_->tiltSlider->setMinimum(min_val);
      ui_->tiltSlider->setMaximum(max_val);
      ui_->tiltSlider->setValue(cur_val);
      ui_->tiltSlider->setSingleStep(step_val);
      ui_->tiltValue->setNum(cur_val);
      ui_->tilt_min->setNum(min_val);
      ui_->tilt_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	zoomUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of zoom control.
  *  Parameter3 : int max_val - maximum value of zoom control.
  *  Parameter4 : int cur_val - current value of zoom control.
  *  Parameter5 : int step_val - step value of zoom control.
  *  Description	:   This function is used to update the zoom UI widgets.
  ************************************************************************************************************/
  void UserInterface::zoomUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->zoomSlider->setEnabled(true);
      ui_->zoomSlider->setTracking(false);
      ui_->zoomValue->setEnabled(true);
      ui_->zoomlabel->setEnabled(true);
      ui_->zoomSlider->setMinimum(min_val);
      ui_->zoomSlider->setMaximum(max_val);
      ui_->zoomSlider->setValue(cur_val);
      ui_->zoomSlider->setSingleStep(step_val);
      ui_->zoomValue->setNum(cur_val);
      ui_->zoom_min->setNum(min_val);
      ui_->zoom_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	hueUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of hue control.
  *  Parameter3 : int max_val - maximum value of hue control.
  *  Parameter4 : int cur_val - current value of hue control.
  *  Parameter5 : int step_val - step value of hue control.
  *  Description	:   This function is used to update the hue UI widgets.
  ************************************************************************************************************/
  void UserInterface::hueUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->hueSlider->setEnabled(true);
      ui_->hueSlider->setTracking(false);
      ui_->hueValue->setEnabled(true);
      ui_->huelabel->setEnabled(true);
      ui_->hueSlider->setMinimum(min_val);
      ui_->hueSlider->setMaximum(max_val);
      ui_->hueSlider->setValue(cur_val);
      ui_->hueSlider->setSingleStep(step_val);
      ui_->hueValue->setNum(cur_val);
      ui_->hue_max->setNum(min_val);
      ui_->hue_min->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	whiteBalanceUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of whitebalance control.
  *  Parameter3 : int max_val - maximum value of whitebalance control.
  *  Parameter4 : int cur_val - current value of whitebalance control.
  *  Parameter5 : int step_val - step value of whitebalance control.
  *  Description	:   This function is used to update the whitebalance UI widgets.
  ************************************************************************************************************/
  void UserInterface::whiteBalanceUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      if(!ui_->wb_auto_checkbox->isEnabled()){
        ui_->whitebalanceSlider->setEnabled(true);
        ui_->whitebalanceSlider->setTracking(false);
        ui_->whitebalanceValue->setEnabled(true);
        ui_->whitebalancelabel->setEnabled(true);
      }
      ui_->whitebalanceSlider->setMinimum(min_val);
      ui_->whitebalanceSlider->setMaximum(max_val);
      ui_->whitebalanceSlider->setValue(cur_val);
      ui_->whitebalanceSlider->setSingleStep(step_val);
      ui_->whitebalanceValue->setNum(cur_val);
      ui_->wb_min->setNum(min_val);
      ui_->wb_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	gammaUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of gamma control.
  *  Parameter3 : int max_val - maximum value of gamma control.
  *  Parameter4 : int cur_val - current value of gamma control.
  *  Parameter5 : int step_val - step value of gamma control.
  *  Description	:   This function is used to update the gamma UI widgets.
  ************************************************************************************************************/
  void UserInterface::gammaUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->gammaSlider->setEnabled(true);
      ui_->gammaSlider->setTracking(false);
      ui_->gammaValue->setEnabled(true);
      ui_->gammalabel->setEnabled(true);
      ui_->gammaSlider->setMinimum(min_val);
      ui_->gammaSlider->setMaximum(max_val);
      ui_->gammaSlider->setValue(cur_val);
      ui_->gammaSlider->setSingleStep(step_val);
      ui_->gammaValue->setNum(cur_val);
      ui_->gamma_min->setNum(min_val);
      ui_->gamma_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	gainUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of gain control.
  *  Parameter3 : int max_val - maximum value of gain control.
  *  Parameter4 : int cur_val - current value of gain control.
  *  Parameter5 : int step_val - step value of gain control.
  *  Description	:   This function is used to update the gain UI widgets.
  ************************************************************************************************************/
  void UserInterface::gainUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->gainSlider->setEnabled(true);
      ui_->gainSlider->setTracking(false);
      ui_->gainValue->setEnabled(true);
      ui_->gainlabel->setEnabled(true);
      ui_->gainSlider->setMinimum(min_val);
      ui_->gainSlider->setMaximum(max_val);
      ui_->gainSlider->setValue(cur_val);
      ui_->gainSlider->setSingleStep(step_val);
      ui_->gainValue->setNum(cur_val);
      ui_->gain_min->setNum(min_val);
      ui_->gain_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	sharpnessUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of Sharpness control.
  *  Parameter3 : int max_val - maximum value of Sharpness control.
  *  Parameter4 : int cur_val - current value of Sharpness control.
  *  Parameter5 : int step_val - step value of Sharpness control.
  *  Description	:   This function is used to update the Sharpness UI widgets.
  ************************************************************************************************************/
  void UserInterface::sharpnessUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->sharpnessSlider->setEnabled(true);
      ui_->sharpnessSlider->setTracking(false);
      ui_->sharpnessValue->setEnabled(true);
      ui_->sharpnesslabel->setEnabled(true);
      ui_->sharpnessSlider->setMinimum(min_val);
      ui_->sharpnessSlider->setMaximum(max_val);
      ui_->sharpnessSlider->setValue(cur_val);
      ui_->sharpnessSlider->setSingleStep(step_val);
      ui_->sharpnessValue->setNum(cur_val);
      ui_->sharpness_min->setNum(min_val);
      ui_->sharpness_max->setNum(max_val);
  }






  /************************************************************************************************************
  *  Name	:	exposureAbsoluteUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of exposureAbsolute control.
  *  Parameter3 : int max_val - maximum value of exposureAbsolute control.
  *  Parameter4 : int cur_val - current value of exposureAbsolute control.
  *  Parameter5 : int step_val - step value of exposureAbsolute control.
  *  Description	:   This function is used to update the exposureAbsolute UI widgets.
  ************************************************************************************************************/
  void UserInterface::exposureAbsoluteUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      if(!ui_->exposure_auto_checkbox->isEnabled()){
        ui_->exposureSlider->setEnabled(true);
        ui_->exposureSlider->setTracking(false);
        ui_->exposureValue->setEnabled(true);
        ui_->exposurelabel->setEnabled(true);
      }
      ui_->exposureSlider->setMinimum(min_val);
      ui_->exposureSlider->setMaximum(max_val);
      ui_->exposureSlider->setValue(cur_val);
      ui_->exposureSlider->setSingleStep(step_val);
      ui_->exposureValue->setNum(cur_val);
      ui_->exp_min->setNum(min_val);
      ui_->exp_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	focusAbsoluteUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of focusAbsolute control.
  *  Parameter3 : int max_val - maximum value of focusAbsolute control.
  *  Parameter4 : int cur_val - current value of focusAbsolute control.
  *  Parameter5 : int step_val - step value of focusAbsolute control.
  *  Description	:   This function is used to update the focusAbsolute UI widgets.
  ************************************************************************************************************/
  void UserInterface::focusAbsoluteUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      if(!ui_->focus_auto_checkbox->isEnabled()){
        ui_->focus_abs_Slider->setEnabled(true);
        ui_->focus_abs_Slider->setTracking(false);
        ui_->focus_abs_Value->setEnabled(true);
        ui_->focus_abs_label->setEnabled(true);
      }
      ui_->focus_abs_Slider->setMinimum(min_val);
      ui_->focus_abs_Slider->setMaximum(max_val);
      ui_->focus_abs_Slider->setValue(cur_val);
      ui_->focus_abs_Slider->setSingleStep(step_val);
      ui_->focus_abs_Value->setNum(cur_val);
      ui_->foc_abs_min->setNum(min_val);
      ui_->foc_abs_max->setNum(max_val);
  }






  /************************************************************************************************************
  *  Name	:	backLightUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of backlight control.
  *  Parameter3 : int max_val - maximum value of backlight control.
  *  Parameter4 : int cur_val - current value of backlight control.
  *  Parameter5 : int step_val - step value of backlight control.
  *  Description	:   This function is used to update the backlight UI widgets.
  ************************************************************************************************************/
  void UserInterface::backLightUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->backlightSlider->setEnabled(true);
      ui_->backlightSlider->setTracking(false);
      ui_->backlightValue->setEnabled(true);
      ui_->backlightlabel->setEnabled(true);
      ui_->backlightSlider->setMinimum(min_val);
      ui_->backlightSlider->setMaximum(max_val);
      ui_->backlightSlider->setValue(cur_val);
      ui_->backlightSlider->setSingleStep(step_val);
      ui_->backlightValue->setNum(cur_val);
      ui_->backlight_min->setNum(min_val);
      ui_->backlight_max->setNum(max_val);
  }






  /************************************************************************************************************
  *  Name	:	focusUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of focus control.
  *  Parameter3 : int max_val - maximum value of focus control.
  *  Parameter4 : int cur_val - current value of focus control.
  *  Parameter5 : int step_val - step value of focus control.
  *  Description	:   This function is used to update the focus UI widgets.
  ************************************************************************************************************/
  void UserInterface::focusUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->focusSlider->setEnabled(true);
      ui_->focusSlider->setTracking(false);
      ui_->focusValue->setEnabled(true);
      ui_->focuslabel->setEnabled(true);
      ui_->focusSlider->setMinimum(min_val);
      ui_->focusSlider->setMaximum(max_val);
      ui_->focusSlider->setValue(cur_val);
      ui_->focusSlider->setSingleStep(step_val);
      ui_->focusValue->setNum(cur_val);
      ui_->focus_min->setNum(min_val);
      ui_->focus_max->setNum(max_val);
  }





  /************************************************************************************************************
  *  Name	:	whiteBalAutoUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of whitebalanceAuto control.
  *  Parameter3 : int max_val - maximum value of whitebalanceAuto control.
  *  Parameter4 : int cur_val - current value of whitebalanceAuto control.
  *  Parameter5 : int step_val - step value of whitebalanceAuto control.
  *  Description	:   This function is used to update the whitebalanceAuto UI widgets.
  ************************************************************************************************************/
  void UserInterface::whiteBalAutoUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->wb_auto_checkbox->setEnabled(true);
      if(cur_val){
        ui_->wb_auto_checkbox->setChecked(true);
        ui_->whitebalanceSlider->setEnabled(false);
        ui_->whitebalanceValue->setEnabled(false);
        ui_->whitebalancelabel->setEnabled(false);
      }else{
        ui_->wb_auto_checkbox->setChecked(false);
        ui_->whitebalanceSlider->setEnabled(true);
        ui_->whitebalanceSlider->setTracking(false);
        ui_->whitebalanceValue->setEnabled(true);
        ui_->whitebalancelabel->setEnabled(true);
      }
  }





  /************************************************************************************************************
  *  Name	:	autoFocusUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of Autofocus control.
  *  Parameter3 : int max_val - maximum value of Autofocus control.
  *  Parameter4 : int cur_val - current value of Autofocus control.
  *  Parameter5 : int step_val - step value of Autofocus control.
  *  Description	:   This function is used to update the Autofocus UI widgets.
  ************************************************************************************************************/
  void UserInterface::autoFocusUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->focus_auto_checkbox->setEnabled(true);
      if(cur_val){
        ui_->focus_auto_checkbox->setChecked(true);
        ui_->focus_abs_Slider->setEnabled(false);
        ui_->focus_abs_Value->setEnabled(false);
        ui_->focus_abs_label->setEnabled(false);
      }else{
        ui_->focus_auto_checkbox->setChecked(false);
        ui_->focus_abs_Slider->setEnabled(true);
        ui_->focus_abs_Slider->setTracking(false);
        ui_->focus_abs_Value->setEnabled(true);
        ui_->focus_abs_label->setEnabled(true);
      }
  }




  /************************************************************************************************************
  *  Name	:	exposureAutoUIupdate.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : int min_val - minimum value of Autoexposure control.
  *  Parameter3 : int max_val - maximum value of Autoexposure control.
  *  Parameter4 : int cur_val - current value of Autoexposure control.
  *  Parameter5 : int step_val - step value of Autoexposure control.
  *  Description	:   This function is used to update the Autoexposure UI widgets.
  ************************************************************************************************************/
  void UserInterface::exposureAutoUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val)
  {
      ui_->exposure_auto_checkbox->setEnabled(true);
      if(cur_val){
        ui_->exposure_auto_checkbox->setChecked(false);
        ui_->exposureSlider->setEnabled(true);
        ui_->exposureSlider->setTracking(false);

        ui_->exposureValue->setEnabled(true);
        ui_->exposurelabel->setEnabled(true);
      }else{
        ui_->exposure_auto_checkbox->setChecked(true);
        ui_->exposureSlider->setEnabled(false);
        ui_->exposureValue->setEnabled(false);
        ui_->exposurelabel->setEnabled(false);
      }
  }




  /************************************************************************************************************
  *  Name	:	setup_format_settings.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter2 : Controls &ctl - object of class Controls.
  *  Parameter3 : int type - This denotes whether to enumerate pixelformat or resolution or framerate.
                  The possible values are:
                  PIX_FORMAT                    1
                  RESOLUTION                    2
                  FPS                           3
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function is used to enumerate format,resolution and framerate and add to ComboBox in UI.
  ************************************************************************************************************/
  int UserInterface:: setup_format_settings(Ui::ImageViewWidget *ui_,Controls &ctl,int type,std::string pixelformat,int width,int height)
  {
    int index;
    std::vector<std::string> temp_list;

    if(ctl.enum_format(type,pixelformat,width,height)<0){
      return FAILURE;
    }
    switch (type) {
      case PIX_FORMAT:
        ui_->format_cmb->clear();
        temp_list=ctl.get_list(PIX_FORMAT);
        for(index=0;index<temp_list.size();index++){
          ui_->format_cmb->addItem(QString::fromStdString(temp_list[index]));
        }
      case RESOLUTION:
        ui_->resolution_cmb->clear();
        temp_list=ctl.get_list(RESOLUTION);
        for(index=0;index<temp_list.size();index++){
          ui_->resolution_cmb->addItem(QString::fromStdString(temp_list[index]));
        }
      case FPS:
        ui_->fps_cmb->clear();
        temp_list=ctl.get_list(FPS);
        for(index=0;index<temp_list.size();index++){
          ui_->fps_cmb->addItem(QString::fromStdString(temp_list[index]));
        }
      default:
        break;
    }
    return SUCCESS;
  }







  /************************************************************************************************************
  *  Name	:	disable_ui_widgets.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Description	:   This function is used to disable all the widgets in UI.
  ************************************************************************************************************/
void UserInterface:: disable_ui_widgets(Ui::ImageViewWidget *ui_)
{
  ui_->brightnessSlider->setEnabled(false);
  ui_->brightnessValue->setEnabled(false);
  ui_->brightnesslabel->setEnabled(false);

  ui_->contrastSlider->setEnabled(false);
  ui_->contrastValue->setEnabled(false);
  ui_->contrastlabel->setEnabled(false);

  ui_->panSlider->setEnabled(false);
  ui_->panValue->setEnabled(false);
  ui_->panlabel->setEnabled(false);

  ui_->saturationSlider->setEnabled(false);
  ui_->saturationValue->setEnabled(false);
  ui_->saturationlabel->setEnabled(false);

  ui_->tiltSlider->setEnabled(false);
  ui_->tiltValue->setEnabled(false);
  ui_->tiltlabel->setEnabled(false);

  ui_->zoomSlider->setEnabled(false);
  ui_->zoomValue->setEnabled(false);
  ui_->zoomlabel->setEnabled(false);

  ui_->backlightSlider->setEnabled(false);
  ui_->backlightValue->setEnabled(false);
  ui_->backlightlabel->setEnabled(false);

  ui_->exposureSlider->setEnabled(false);
  ui_->exposureValue->setEnabled(false);
  ui_->exposurelabel->setEnabled(false);

  ui_->focusSlider->setEnabled(false);
  ui_->focusValue->setEnabled(false);
  ui_->focuslabel->setEnabled(false);

  ui_->focus_abs_Slider->setEnabled(false);
  ui_->focus_abs_Value->setEnabled(false);
  ui_->focus_abs_label->setEnabled(false);

  ui_->gainSlider->setEnabled(false);
  ui_->gainValue->setEnabled(false);
  ui_->gainlabel->setEnabled(false);

  ui_->gammaSlider->setEnabled(false);
  ui_->gammaValue->setEnabled(false);
  ui_->gammalabel->setEnabled(false);

  ui_->hueSlider->setEnabled(false);
  ui_->hueValue->setEnabled(false);
  ui_->huelabel->setEnabled(false);

  ui_->sharpnessSlider->setEnabled(false);
  ui_->sharpnessValue->setEnabled(false);
  ui_->sharpnesslabel->setEnabled(false);

  ui_->whitebalanceSlider->setEnabled(false);
  ui_->whitebalanceValue->setEnabled(false);
  ui_->whitebalancelabel->setEnabled(false);

  ui_->wb_auto_checkbox->setEnabled(false);
  ui_->focus_auto_checkbox->setEnabled(false);
  ui_->exposure_auto_checkbox->setEnabled(false);

  ui_->brightness_min->setEnabled(false);
  ui_->brightness_max->setEnabled(false);
  ui_->contrast_min->setEnabled(false);
  ui_->contrast_max->setEnabled(false);
  ui_-> saturation_min->setEnabled(false);
  ui_->saturation_max ->setEnabled(false);
  ui_-> pan_min->setEnabled(false);
  ui_->pan_max ->setEnabled(false);
  ui_-> tilt_min->setEnabled(false);
  ui_-> tilt_max->setEnabled(false);
  ui_-> zoom_min->setEnabled(false);
  ui_->zoom_max ->setEnabled(false);
  ui_-> hue_min->setEnabled(false);
  ui_-> hue_max->setEnabled(false);
  ui_-> wb_min->setEnabled(false);
  ui_->wb_max ->setEnabled(false);
  ui_-> gamma_min->setEnabled(false);
  ui_-> gamma_max->setEnabled(false);
  ui_-> gain_min->setEnabled(false);
  ui_-> gain_max->setEnabled(false);
  ui_->sharpness_min ->setEnabled(false);
  ui_->sharpness_max ->setEnabled(false);
  ui_-> exp_min->setEnabled(false);
  ui_->exp_max ->setEnabled(false);
  ui_-> foc_abs_min->setEnabled(false);
  ui_-> foc_abs_max->setEnabled(false);
  ui_-> backlight_min->setEnabled(false);
  ui_-> backlight_max->setEnabled(false);
  ui_->focus_min ->setEnabled(false);
  ui_-> focus_max->setEnabled(false);
  ui_->reset_button->setEnabled(false);
}






/************************************************************************************************************
*  Name	:	update_slider_value.
*  Parameter1 : int value - Control value.
*  Parameter2 : uint32_t id - Control ID.
*  Parameter3 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
*  Parameter4 : Controls &ctl - object of class Controls.
*  Parameter5 : struct controlid *ids - structure pointer of struct controlid.
*  Description	:   This function is used to update the respected slider value.
************************************************************************************************************/
  void UserInterface::update_slider_value(int value,uint32_t Id,Ui::ImageViewWidget *ui_,Controls &ctl,struct controlid ids)
  {
    if(Id==ids.BrightnessId){
          ui_->brightnessValue->setNum(value);
    }else if(Id==ids.ContrastId){
        ui_->contrastValue->setNum(value);
    }else if(Id==ids.SaturationId){
        ui_->saturationValue->setNum(value);
    }else if(Id==ids.PanAbsoluteId){
        ui_->panValue->setNum(value);
    }else if(Id==ids.TiltAbsoluteId){
        ui_->tiltValue->setNum(value);
    }else if(Id==ids.ZoomAbsoluteId){
        ui_->zoomValue->setNum(value);
    }else if(Id==ids.HueId){
        ui_->hueValue->setNum(value);
    }else if(Id==ids.WhiteBalanceTemperatureId){
        ui_->whitebalanceValue->setNum(value);
    }else if(Id==ids.GammaId){
        ui_->gammaValue->setNum(value);
    }else if(Id==ids.GainId){
        ui_->gainValue->setNum(value);
    }else if(Id==ids.SharpnessId){
        ui_->sharpnessValue->setNum(value);
    }else if(Id==ids.ExposureAbsoluteId){
        ui_->exposureValue->setNum(value);
    }else if(Id==ids.FocusAbsoluteId){
        ui_->focus_abs_Value->setNum(value);
    }else if(Id==ids.BacklightCompensationId){
        ui_->backlightValue->setNum(value);
    }else if(Id==ids.FocusId){
        ui_->focusValue->setNum(value);
    }else{
      ROS_INFO("Id doesn't match:%d",Id);
    }
  }






  /************************************************************************************************************
  *  Name	:	update_checkbox_setting.
  *  Parameter1 : int value - Control value.
  *  Parameter2 : uint32_t id - Control ID.
  *  Parameter3 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Parameter4 : Controls &ctl - object of class Controls.
  *  Parameter5 : struct controlid *ids - structure pointer of struct controlid.
  *  Description	:   This function is used to update the respected CheckBox.
  ************************************************************************************************************/
  void UserInterface::update_checkbox_setting(uint32_t Id,int value,Ui::ImageViewWidget *ui_,Controls &ctl,struct controlid ids)
  {
    if(Id==ids.WhiteBalanceTemperatureAutoId){
      ui_->whitebalanceSlider->setEnabled((bool)!value);
      ui_->whitebalanceValue->setEnabled((bool)!value);
      ui_->whitebalancelabel->setEnabled((bool)!value);
    }else if(Id==ids.FocusAutoId){
      ui_->focus_abs_Slider->setEnabled((bool)!value);
      ui_->focus_abs_Value->setEnabled((bool)!value);
      ui_->focus_abs_label->setEnabled((bool)!value);
    }else if(Id==ids.ExposureAutoId){
      ui_->exposureSlider->setEnabled((bool)value);
      ui_->exposureValue->setEnabled((bool)value);
      ui_->exposurelabel->setEnabled((bool)value);
    }
  }



  /************************************************************************************************************
  *  Name	:	update_image_format_cmb.
  *  Parameter1 : Ui::ImageViewWidget *ui_ - Object of user interface widget.
  *  Description	:   This function is used to update the image format ComboBox.
  ************************************************************************************************************/
  void UserInterface::update_image_format_cmb(Ui::ImageViewWidget *ui_)
  {
    ui_->image_format_cmb->addItem("jpg");
    ui_->image_format_cmb->addItem("bmp");
    ui_->image_format_cmb->addItem("png");
    ui_->image_format_cmb->addItem("raw");

  }

}// end of namespace rqt_cam
