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

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <linux/videodev2.h>
/* MACRO FOR BAYER10 TO RGB24 */

#define Bay(x, y, w) bayerIRBuffer[(x) + (w) * (y)]
#define CLIP(x) (((x) >= 255)? 255 : (x))

namespace rqt_cam{

  class Conversion
  {
    private:
      unsigned short int *bayerIRBuffer;
    public:
      void ConvertY12toY8(uchar * Y12Buff,int height,int width, unsigned char *Y8Buff);

      void ConvertY16toY8(uchar * Y16Buff,int height,int width,unsigned char *Y8Buff);

      void ConvertY16toY8for20CUG(uint16_t * Y16Buff,int height,int width,unsigned char *Y8Buff);

      void ConvertY16toRGB(void * Y16Buff,int height,int width,unsigned char  *RGBDestBuffer);

      void ConvertBY8toRGB(uint8_t *bayer, int width, int height, uint8_t *bgr);

      void convert_border_bayer_line_to_bgr24( uint8_t* bayer, uint8_t* adjacent_bayer,
          uint8_t *bgr, int width, uint8_t start_with_green, uint8_t blue_line);
  };

}
