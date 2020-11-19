/*
 * The MIT License (MIT)
 *
 * ECAM_V4L2
 * Copyright (c) 2020 e-consystems
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
#define PIX_FORMAT												1
#define RESOLUTION												2
#define FPS																3
#define MAX_FRAME_SIZE_SUPPORT						16
#define MAX_PIXELFORMAT_SUPPORT						16
#define MAX_FRMINVAL_SUPPORT							10
#define DISABLE														0
#define ENABLE														1
#define SUCCESS														0
#define FAILURE														-1
#define DEV_UNPLUGGED											-2
#include <string>
#include <vector>
#include <asm/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "ecam_v4l2/image.h"
#include <ros/topic_manager.h>
namespace ecam_v4l2
{
	class Camera
	{
		private:
			class Buffer
			{
			  public:
				uint8_t *start;
				size_t length;
			};// end of class Buffer

			class V4L2_structure
			{
				public:
					struct v4l2_frmsizeenum 	fsize[MAX_FRAME_SIZE_SUPPORT];
					struct v4l2_fmtdesc		ffmt[MAX_PIXELFORMAT_SUPPORT];
					struct v4l2_frmivalenum		frminterval;
					struct v4l2_fract		desc_frm_inval[MAX_FRMINVAL_SUPPORT];
					struct v4l2_format		stream_fmt;
					struct v4l2_streamparm 		cam_parm;
					struct v4l2_requestbuffers reqbuf;
					struct v4l2_capability cap;
					struct v4l2_buffer buffer;
			};// end of class Format
			int cam_fd;
			int subscribers_count;
			int width,height;
			int numerator,denominator;
			bool streamon; // flag to indicate camera is streaming
			bool init; //flag to indicate initialisation
			std::string pixelformat;
			std::vector<Buffer> buffers;
			std::string camera_name;
			std::string video_node;

// private member functions
			void enqueue(int index);
// public of class Camera
		public:

			V4L2_structure v4l2; //obj of class Format
// Member function of class Camera
			Camera();

			int open_device(const std::string &device);

			void cam_init();

			void enum_pixelformat(std::vector<std::string> *list,std::string *str);

			void enum_resolution(std::string pixelformat,
													 std::vector<std::string> *list,
													 std::string *str);

			void enum_framerate(std::string pixelformat,int width,int height,
													std::vector<std::string> *list,std::string *str);

			int set_format(const std::string& fourcc,int input_width,int input_height);

			void get_format(std::string *PixelFormat,int *Height,int *Width,
											int *Numerator,int *Denominator,int type);

			int set_framerate(unsigned input_numerator, unsigned input_denominator);

			int request_buffer();

			int stream_on();

			int capture(ecam_v4l2::image *image);

			int stream_off();

			int clean_buffer();

			void munmap_buffers();

			void close_device();


			bool onInit();

			void set_init(bool value);

			int sub_count();

			bool isStreamOn();

			int xioctl(unsigned cmd, void *arg);

			void get_four_character_code(int32_t pix_fmt,std::string *pixelformat);

			void set_camera_name(std::string name,std::string node_name);

			std::string get_camera_name();

			void get_node_name(std::string *dev_node_name);

	};// end of class Camera

}// ecam_v4l2
#endif //V4L2_H
