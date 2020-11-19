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
#include <rqt_cam/conversion.h>





namespace rqt_cam{





    /************************************************************************************************************
    *  Name	:	ConvertY12toY8.
    *  Parameter1 : uchar * Y12Buff - buffer in which is Y12 data is stored.
    *  Parameter2 : int height - height of the frame.
    *  Parameter3 : int width - width of the frame.
    *  Parameter4 : unsigned char  *Y8Buff - buffer in which is Y8 data is to be stored
    *  Description	: This function is to convert Y12 data to Y8 data.
    ************************************************************************************************************/
    void Conversion::ConvertY12toY8(uchar * Y12Buff,int height,int width, unsigned char  *Y8Buff)
    {
/* Y12 to Y8 conversion logic

    Y12data:         1st 12 bits                                       2nd 12 bits
    |<--------------------------------------------->|<--------------------------------------------->|
    | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D | D |
    |<----------------------------->|<------------->|<----------------------------->|<------------->|
    Y8data:   1st 8byte              we will skip 		2nd byte              we will skip
				     this nibble                                        this nibble                        

*/
      int h,w,byte = 0;
      for (h = 0; h < height;h++){
        for (w = 0;w < width;w += 2){
          *(Y8Buff+(h*width + w)) = Y12Buff[byte];
          *(Y8Buff+(h*width + w + 1)) = Y12Buff[byte + 1];
          byte += 3;
        }
      }
    }





    /************************************************************************************************************
    *  Name	:	ConvertY16toY8for20CUG.
    *  Parameter1 : uint16_t * Y16Buff - buffer in which is Y16 data is stored.
    *  Parameter2 : int height - height of the frame.
    *  Parameter3 : int width - width of the frame.
    *  Parameter4 : unsigned char  *Y8Buff - buffer in which is Y8 data is to be stored
    *  Description	: This function is to convert Y16 data to Y8 data for See3CAM_20CUG camera.
    ************************************************************************************************************/
    void Conversion::ConvertY16toY8for20CUG(uint16_t * Y16Buff,int height,int width, unsigned char  *Y8Buff)
    {
/*Y16 to Y8 conversion logic
    In this conversion logic 10 bit data is mapped to 8 bit data.
    This is done by multiplying a factor with the 16 bit data.
    The factor is derived as follows:
        2^10 = 1024.
        The maximum value that can be stored in 8 bit is 255.
        Therefore  255/1024 = 0.2490234375.
        0.2490234375 is the multiplying factor.
*/
        for(__u32 cnt=0; cnt<(width * height); cnt++) {
            *Y8Buff++ =(Y16Buff[cnt] * 0.2490234375);
        }
    }





    /************************************************************************************************************
    *  Name	:	ConvertY16toY8.
    *  Parameter1 : uchar * Y16Buff - buffer in which is Y16 data is stored.
    *  Parameter2 : int height - height of the frame.
    *  Parameter3 : int width - width of the frame.
    *  Parameter4 : unsigned char *Y8Buff - buffer in which is Y8 data is to be stored
    *  Description	: This function is to convert Y16 data to Y8 data.
    ************************************************************************************************************/
    void Conversion::ConvertY16toY8(uchar * Y16Buff,int height,int width, unsigned char *Y8Buff)
    {
/* Y12 to Y8 conversion logic

    Y12data:                  1st byte
    |<------------------------------------------------------------->|
    | D | D | D | D | * | * | * | * | * | * | * | * | D | D | D | D |
    |<----------------------------->|<----------------------------->|
            1st 8bit &  0xF0 >> 4         2nd 8bit & 0x0F << 4
                                     |
                                    \/
                    | D | D | D | D | D | D | D | D |
                    |<----------------------------->|
    Y8data:                     1st byte

*/
      for(__u32 cnt=0; cnt<(width * height*2); cnt=cnt+2) {
        *Y8Buff++ = (((*(Y16Buff+cnt) & 0xF0) >> 4) | (*(Y16Buff+(cnt+1)) & 0x0F) << 4);
      }

    }






    /************************************************************************************************************
    *  Name	:	ConvertY16toRGB.
    *  Parameter1 : void * Y16Buff - buffer in which is Y16 data is stored.
    *  Parameter2 : int height - height of the frame.
    *  Parameter3 : int width - width of the frame.
    *  Parameter4 : unsigned char *RGBDestBuffer  - buffer in which is RGB data is to be stored
    *  Description	: This function is to convert Y16 data to RGB data.
    ************************************************************************************************************/
    void Conversion::ConvertY16toRGB(void * Y16Buff,int height,int width,unsigned char *RGBDestBuffer)
    {
      bayerIRBuffer = (unsigned short int *)malloc(width * height* 2);
      memcpy(bayerIRBuffer, Y16Buff, (width*height*2));
      for(__u32 x = 0; x < width; x += 2)  /* Nearest neighbour interpolation algorithm - y16 to RGB24 conversion */
      {
          for(__u32 y = 0; y < height; y += 2)
          {
            *(RGBDestBuffer+(2 + 3 * ((x) + (width) * (y))))     = *(RGBDestBuffer+(2 + 3 * ((x+1) + (width) * (y)))) =
            *(RGBDestBuffer+(2 + 3 * ((x) + (width) * (y+1)))) = *(RGBDestBuffer+(2 + 3 * ((x+1) + (width) * (y+1))))
                = CLIP(Bay(x, y, width));
            *(RGBDestBuffer+(1 + 3 * ((x) + (width) * (y))))     = *(RGBDestBuffer+(1 + 3 * ((x+1) + (width) * (y)))) =
            *(RGBDestBuffer+(1 + 3 * ((x) + (width) * (y+1)))) = *(RGBDestBuffer+(1 + 3 * ((x+1) + (width) * (y+1))))
                = CLIP(Bay(x + 1, y, width));
            *(RGBDestBuffer+(0 + 3 * ((x) + (width) * (y))))     = *(RGBDestBuffer+(0 + 3 * ((x+1) + (width) * (y)))) =
            *(RGBDestBuffer+(0 + 3 * ((x) + (width) * (y+1)))) = *(RGBDestBuffer+(0 + 3 * ((x+1) + (width) * (y+1))))
                = CLIP(Bay(x + 1, y + 1, width));
          }
      }
      free(bayerIRBuffer);
    }







    /************************************************************************************************************
    *  Name	:	convert_border_bayer_line_to_bgr24.
    *  Parameter1 :  uint8_t* bayer - pointer to bayerIR buffer
    *  Parameter2 :  uint8_t* adjacent_bayer - pointer to second and second last line of bayerIR buffer.
    *  Parameter3 :  uint8_t *bgr - pointer to rgb data buffer.
    *  Parameter4 : int width - width of the frame.
    *  Parameter5 : uint8_t start_with_green - flag to denote whether to start with green pixel
    *  Parameter6 : uint8_t blue_line -flag to denote whether to start with blue line.
    *  Description	: This function is to convert the border line of bayerIR frame to rgb data.
    ************************************************************************************************************/
    void  Conversion::convert_border_bayer_line_to_bgr24( uint8_t* bayer, uint8_t* adjacent_bayer,
        uint8_t *bgr, int width, uint8_t start_with_green, uint8_t blue_line)
    {
        int t0, t1;

        if (start_with_green)
        {
        /* First pixel */
            if (blue_line)
            {
                *bgr++ = bayer[1];
                *bgr++ = bayer[0];
                *bgr++ = adjacent_bayer[0];
            }
            else
            {
                *bgr++ = adjacent_bayer[0];
                *bgr++ = bayer[0];
                *bgr++ = bayer[1];
            }
            /* Second pixel */
            t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
            t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
            if (blue_line)
            {
                *bgr++ = bayer[1];
                *bgr++ = t0;
                *bgr++ = t1;
            }
            else
            {
                *bgr++ = t1;
                *bgr++ = t0;
                *bgr++ = bayer[1];
            }
            bayer++;
            adjacent_bayer++;
            width -= 2;
        }
        else
        {
            /* First pixel */
            t0 = (bayer[1] + adjacent_bayer[0] + 1) >> 1;
            if (blue_line)
            {
                *bgr++ = bayer[0];
                *bgr++ = t0;
                *bgr++ = adjacent_bayer[1];
            }
            else
            {
                *bgr++ = adjacent_bayer[1];
                *bgr++ = t0;
                *bgr++ = bayer[0];
            }
            width--;
        }

        if (blue_line)
        {
            for ( ; width > 2; width -= 2)
            {
                t0 = (bayer[0] + bayer[2] + 1) >> 1;
                *bgr++ = t0;
                *bgr++ = bayer[1];
                *bgr++ = adjacent_bayer[1];
                bayer++;
                adjacent_bayer++;

                t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
                t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
                *bgr++ = bayer[1];
                *bgr++ = t0;
                *bgr++ = t1;
                bayer++;
                adjacent_bayer++;
            }
        }
        else
        {
            for ( ; width > 2; width -= 2)
            {
                t0 = (bayer[0] + bayer[2] + 1) >> 1;
                *bgr++ = adjacent_bayer[1];
                *bgr++ = bayer[1];
                *bgr++ = t0;
                bayer++;
                adjacent_bayer++;

                t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
                t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
                *bgr++ = t1;
                *bgr++ = t0;
                *bgr++ = bayer[1];
                bayer++;
                adjacent_bayer++;
            }
        }

        if (width == 2)
        {
            /* Second to last pixel */
            t0 = (bayer[0] + bayer[2] + 1) >> 1;
            if (blue_line)
            {
                *bgr++ = t0;
                *bgr++ = bayer[1];
                *bgr++ = adjacent_bayer[1];
            }
            else
            {
                *bgr++ = adjacent_bayer[1];
                *bgr++ = bayer[1];
                *bgr++ = t0;
            }
            /* Last pixel */
            t0 = (bayer[1] + adjacent_bayer[2] + 1) >> 1;
            if (blue_line)
            {
                *bgr++ = bayer[2];
                *bgr++ = t0;
                *bgr++ = adjacent_bayer[1];
            }
            else
            {
                *bgr++ = adjacent_bayer[1];
                *bgr++ = t0;
                *bgr++ = bayer[2];
            }
        }
        else
        {
            /* Last pixel */
            if (blue_line)
            {
                *bgr++ = bayer[0];
                *bgr++ = bayer[1];
                *bgr++ = adjacent_bayer[1];
            }
            else
            {
                *bgr++ = adjacent_bayer[1];
                *bgr++ = bayer[1];
                *bgr++ = bayer[0];
            }
        }
    }






    /************************************************************************************************************
    *  Name	:	ConvertBY8toRGB.
    *  Parameter1 : uint8_t * bayer - buffer in which is bayer8 data is stored.
    *  Parameter2 : int height - height of the frame.
    *  Parameter3 : int width - width of the frame.
    *  Parameter4 : uint8_t *bgr  - buffer in which is RGB data is to be stored
    *  Description	: This function is to convert Y16 data to RGB data.
    ************************************************************************************************************/
    void Conversion::ConvertBY8toRGB(uint8_t *bayer, int width, int height, uint8_t *bgr )
    {
      uint8_t start_with_green=1;
      uint8_t blue_line = 1;
        /* render the first line */
        convert_border_bayer_line_to_bgr24(bayer, bayer + width, bgr, width,
            start_with_green, blue_line);
        bgr += width * 3;

        /* reduce height by 2 because of the special case top/bottom line */
        for (height -= 2; height; height--)
        {
            int t0, t1;
            /* (width - 2) because of the border */
            uint8_t *bayerEnd = bayer + (width - 2);

            if (start_with_green)
            {
                /* OpenCV has a bug in the next line, which was
                t0 = (bayer[0] + bayer[width * 2] + 1) >> 1; */
                t0 = (bayer[1] + bayer[width * 2 + 1] + 1) >> 1;
                /* Write first pixel */
                t1 = (bayer[0] + bayer[width * 2] + bayer[width + 1] + 1) / 3;
                if (blue_line)
                {
                    *bgr++ = t0;
                    *bgr++ = t1;
                    *bgr++ = bayer[width];
                }
                else
                {
                    *bgr++ = bayer[width];
                    *bgr++ = t1;
                    *bgr++ = t0;
                }

                /* Write second pixel */
                t1 = (bayer[width] + bayer[width + 2] + 1) >> 1;
                if (blue_line)
                {
                    *bgr++ = t0;
                    *bgr++ = bayer[width + 1];
                    *bgr++ = t1;
                }
                else
                {
                    *bgr++ = t1;
                    *bgr++ = bayer[width + 1];
                    *bgr++ = t0;
                }
                bayer++;
            }
            else
            {
                /* Write first pixel */
                t0 = (bayer[0] + bayer[width * 2] + 1) >> 1;
                if (blue_line)
                {
                    *bgr++ = t0;
                    *bgr++ = bayer[width];
                    *bgr++ = bayer[width + 1];
                }
                else
                {
                    *bgr++ = bayer[width + 1];
                    *bgr++ = bayer[width];
                    *bgr++ = t0;
                }
            }

            if (blue_line)
            {
                for (; bayer <= bayerEnd - 2; bayer += 2)
                {
                    t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
                        bayer[width * 2 + 2] + 2) >> 2;
                    t1 = (bayer[1] + bayer[width] +
                        bayer[width + 2] + bayer[width * 2 + 1] +
                        2) >> 2;
                    *bgr++ = t0;
                    *bgr++ = t1;
                    *bgr++ = bayer[width + 1];

                    t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
                    t1 = (bayer[width + 1] + bayer[width + 3] +
                        1) >> 1;
                    *bgr++ = t0;
                    *bgr++ = bayer[width + 2];
                    *bgr++ = t1;
                }
            }
            else
            {
                for (; bayer <= bayerEnd - 2; bayer += 2)
                {
                    t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
                        bayer[width * 2 + 2] + 2) >> 2;
                    t1 = (bayer[1] + bayer[width] +
                        bayer[width + 2] + bayer[width * 2 + 1] +
                        2) >> 2;
                    *bgr++ = bayer[width + 1];
                    *bgr++ = t1;
                    *bgr++ = t0;

                    t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
                    t1 = (bayer[width + 1] + bayer[width + 3] +
                        1) >> 1;
                    *bgr++ = t1;
                    *bgr++ = bayer[width + 2];
                    *bgr++ = t0;
                }
            }

            if (bayer < bayerEnd)
            {
                /* write second to last pixel */
                t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
                    bayer[width * 2 + 2] + 2) >> 2;
                t1 = (bayer[1] + bayer[width] +
                    bayer[width + 2] + bayer[width * 2 + 1] +
                    2) >> 2;
                if (blue_line)
                {
                    *bgr++ = t0;
                    *bgr++ = t1;
                    *bgr++ = bayer[width + 1];
                }
                else
                {
                    *bgr++ = bayer[width + 1];
                    *bgr++ = t1;
                    *bgr++ = t0;
                }
                /* write last pixel */
                t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
                if (blue_line)
                {
                    *bgr++ = t0;
                    *bgr++ = bayer[width + 2];
                    *bgr++ = bayer[width + 1];
                }
                else
                {
                    *bgr++ = bayer[width + 1];
                    *bgr++ = bayer[width + 2];
                    *bgr++ = t0;
                }
                bayer++;
            }
            else
            {
                /* write last pixel */
                t0 = (bayer[0] + bayer[width * 2] + 1) >> 1;
                t1 = (bayer[1] + bayer[width * 2 + 1] + bayer[width] + 1) / 3;
                if (blue_line)
                {
                    *bgr++ = t0;
                    *bgr++ = t1;
                    *bgr++ = bayer[width + 1];
                }
                else
                {
                    *bgr++ = bayer[width + 1];
                    *bgr++ = t1;
                    *bgr++ = t0;
                }
            }

            /* skip 2 border pixels */
            bayer += 2;

            blue_line = !blue_line;
            start_with_green = !start_with_green;
        }

        /* render the last line */
        convert_border_bayer_line_to_bgr24(bayer + width, bayer, bgr, width,
            !start_with_green, !blue_line);
    }


} // end of namespace rqt_cam
