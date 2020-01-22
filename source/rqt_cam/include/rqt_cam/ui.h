#ifndef UI_H
#define UI_H

#include "rqt_cam/controls.h"
#include "rqt_cam/query_control.h"
#include "rqt_cam/set_control.h"
#include <ui_image_view.h>

namespace rqt_cam{

  class UserInterface
  {
    private:
      int controls_size;
    public:
      UserInterface();
      int setup_uvc_settings(Ui::ImageViewWidget *ui_,Controls &ctl,struct controlid *ids);
      void reset_uvc_settings(Ui::ImageViewWidget *ui_,Controls &ctl);
      int setup_format_settings(Ui::ImageViewWidget *ui_,Controls &ctl,int type,std::string pixelformat,int width,int height);
      void disable_ui_widgets(Ui::ImageViewWidget *ui_);
      void update_slider_value(int value,uint32_t Id,Ui::ImageViewWidget *ui_,Controls &ctl,struct controlid ids);
      void update_checkbox_setting(uint32_t Id,int value,Ui::ImageViewWidget *ui_,Controls &ctl,struct controlid ids);
      void update_image_format_cmb(Ui::ImageViewWidget *ui_);
      void brightnessUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void contrastUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void saturationUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void panUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void tiltUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void zoomUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void hueUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void whiteBalanceUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void gammaUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void gainUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void sharpnessUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void exposureAbsoluteUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void focusAbsoluteUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void backLightUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void focusUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void whiteBalAutoUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void autoFocusUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);
      void exposureAutoUIupdate(Ui::ImageViewWidget *ui_,int min_val,int max_val,int cur_val,int step_val);

  };// end of class UserInterface

}// end of namespace rqt_cam
#endif // UI_H
