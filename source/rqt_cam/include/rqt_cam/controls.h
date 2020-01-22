#ifndef CONTROLS_H
#define CONTROLS_H
#include "rqt_cam/query_control.h"
#include "rqt_cam/set_control.h"
#include "rqt_cam/set_format.h"
#include "rqt_cam/enum_format.h"
#include "rqt_cam/camera.h"
#include <QString>
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
namespace rqt_cam{
  // Structure to store control ID values.
  struct controlid
  {
    uint32_t BrightnessId;
    uint32_t ContrastId;
    uint32_t SaturationId;
    uint32_t PanAbsoluteId;
    uint32_t TiltAbsoluteId;
    uint32_t ZoomAbsoluteId;
    uint32_t HueId;
    uint32_t WhiteBalanceTemperatureId;
    uint32_t GammaId;
    uint32_t GainId;
    uint32_t SharpnessId;
    uint32_t ExposureAbsoluteId;
    uint32_t FocusAbsoluteId;
    uint32_t BacklightCompensationId;
    uint32_t RawBitsPerPixelId;
    uint32_t LedFrequencyId;
    uint32_t FocusId;
    uint32_t WhiteBalanceTemperatureAutoId;
    uint32_t FocusAutoId;
    uint32_t ExposureAutoId;
  };

  class Controls
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
      std::vector<queryctrl> ctrl; //list of controls available and their values will be stored.
      std::string camera_name;
    public:

// Service client call functions
      int select_camera(std::string cam_name,bool shutdown);

      int query_control();

      int set_control(uint32_t id,int value);

      int enum_format(int type,std::string pix_fmt,int width,int height);

      int set_format(std::string format,int width,int height,int numerator, int denominator);



      int get_value(int index,int type);

      void get_control_name(int index,std::string *control_name);

      std::vector<std::string> get_list(int type);

      void clear_controls_list();
  };
}// end of namespace rqt_cam

#endif
