/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 ThundeRatz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef V4L2_H
#define V4L2_H
#define PIX_FORMAT                				1
#define RESOLUTION                				2
#define FPS                       				3
#define MAX_FRAME_SIZE_SUPPORT						16
#define MAX_PIXELFORMAT_SUPPORT 					16
#define MAX_FRMINVAL_SUPPORT							10
#define DISABLE 													0
#define ENABLE 														1
#define FAILURE 													-1
#define SUCCESS 													0
#include <string>
#include <vector>
#include <asm/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "ecam_v4l2/image.h"

namespace ecam_v4l2
{
	class Camera
	{
		private:
			int cam_fd;
			class Buffer
			{
			  public:
				uint8_t *start;
				size_t length;
			};// end of class Buffer

			std::vector<Buffer> buffers;
			bool streamon; // flag to indicate camera is streaming
			class Format
			{
				public:
					struct v4l2_frmsizeenum 	fsize[MAX_FRAME_SIZE_SUPPORT];
					struct v4l2_fmtdesc		ffmt[MAX_PIXELFORMAT_SUPPORT];
					struct v4l2_frmivalenum		frminterval;
					struct v4l2_fract		desc_frm_inval[MAX_FRMINVAL_SUPPORT];
					struct v4l2_format		fmt,stream_fmt;
					struct v4l2_streamparm 		cam_parm;
			};// end of class Format
			int width,height;
			int numerator,denominator;
			std::string pixelformat;
			bool init; //flag to indicate initialisation

// public of class Camera
		public:

			Format frmt; //obj of class Format
// Member function of class Camera
			Camera();

			int open_device(const std::string &device);

			void init_format();

			int set_format(const std::string& fourcc,int width,int height);

			void get_format(std::string *PixelFormat,int *Height,int *Width,int type);

			int set_framerate(unsigned numerator, unsigned denominator);

			int request_buffer();

			int stream_on();

			void enqueue(int index);

			int capture(ecam_v4l2::image *image);

			int empty_image(int buffer_index,ecam_v4l2::image *image);

			int stream_off();

			int clean_buffer();

			void close_device();


			bool onInit();

			bool isStreamOn();

			int xioctl(unsigned cmd, void *arg);

			void get_four_character_code(int32_t pix_fmt,std::string *pixelformat);

	};// end of class Camera

}// ecam_v4l2
#endif //V4L2_H
