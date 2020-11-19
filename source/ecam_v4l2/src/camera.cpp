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
#include "ecam_v4l2/V4l2.h"

namespace ecam_v4l2
{


  // Constructor of class Camera
  Camera::Camera()
  {
    streamon=false;
    cam_fd=-1;
    init=true;
    subscribers_count = 0;
  }

  /*****************************************************************************
   *  Name	:	open_device.
   *  Parameter1 : const std::string &device - Name of the device node,
   *                                           For ex. /dev/video0
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function is to open the camera file descriptor.
   ****************************************************************************/
  int Camera::open_device(const std::string &device)
  {
    // checking if camera is not already open.
    if(cam_fd==-1){
      cam_fd=open(device.c_str(),O_RDWR|O_NONBLOCK);
    }
    // if open camera failed cam_fd will be equal to -1
    if(cam_fd<0){
      ROS_INFO("Open device Failed");
      return FAILURE;
    }
    memset(&v4l2.cap,0,sizeof(v4l2.cap));
    ioctl(cam_fd, VIDIOC_QUERYCAP, &v4l2.cap);

    if (!(v4l2.cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
      ROS_INFO("Device does not support video capture");
      return FAILURE;
    }

    if (!(v4l2.cap.capabilities & V4L2_CAP_STREAMING)){
      ROS_INFO("Device does not support streaming I/O");
      return FAILURE;
    }
    return SUCCESS;
  }




  /*****************************************************************************
   *  Name	:	cam_init.
   *  Description	:   This function is to setup the camera to streamon,
   *                  once it is selected.
   ****************************************************************************/
  void Camera::cam_init()
  {
    memset(&v4l2.stream_fmt,0,sizeof(v4l2.stream_fmt));
    v4l2.stream_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if(ioctl(cam_fd, VIDIOC_G_FMT, &v4l2.stream_fmt)<0){
      ROS_INFO("VIDIOC_S_FMT failed");
      return;
    }

    get_four_character_code(v4l2.stream_fmt.fmt.pix.pixelformat,&pixelformat);
    set_format(pixelformat,v4l2.stream_fmt.fmt.pix.width,
                           v4l2.stream_fmt.fmt.pix.height);

    memset(&v4l2.cam_parm,0,sizeof(v4l2.cam_parm));
    v4l2.cam_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(cam_fd, VIDIOC_G_PARM, &v4l2.cam_parm);
    if (!(v4l2.cam_parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)){
      ROS_WARN("V4L2_CAP_TIMEPERFRAME not supported by driver");
      ROS_WARN("Leaving frame rate unchanged");
      return;
    }
    numerator = v4l2.cam_parm.parm.capture.timeperframe.numerator;
    denominator = v4l2.cam_parm.parm.capture.timeperframe.denominator;
    request_buffer();
    stream_on();
    init=true;
  }



  void Camera::enum_pixelformat(std::vector<std::string> *list,std::string *str)
  {

    for (int cnt=0 ;cnt< MAX_PIXELFORMAT_SUPPORT; cnt++){
      memset(&v4l2.ffmt[cnt],0,sizeof(v4l2.ffmt[cnt]));
      v4l2.ffmt[cnt].index		= cnt;
      v4l2.ffmt[cnt].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if(0 == xioctl(VIDIOC_ENUM_FMT, &v4l2.ffmt[cnt])) {
        get_four_character_code(v4l2.ffmt[cnt].pixelformat,str);
        list->push_back(*str);
      }
      else{
        break;
      }
    }
    memset(&v4l2.stream_fmt,0,sizeof(v4l2.stream_fmt));
    v4l2.stream_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if(0 == xioctl(VIDIOC_G_FMT, &v4l2.stream_fmt)) {
      get_four_character_code(v4l2.stream_fmt.fmt.pix.pixelformat,str);
    }
  }


  void Camera::enum_resolution(std::string pixelformat,
                               std::vector<std::string> *list,
                               std::string *str)
  {
    std::stringstream temp_stream;
    for (int cnt=0 ;cnt< MAX_FRAME_SIZE_SUPPORT; cnt++){

      memset(&v4l2.fsize[cnt],0,sizeof(v4l2.fsize[cnt]));

      temp_stream.str("");    // clearing temp_stream
      v4l2.fsize[cnt].index		= cnt;
      v4l2.fsize[cnt].pixel_format	= v4l2_fourcc(pixelformat[0],
                                                  pixelformat[1],
                                                  pixelformat[2],
                                                  pixelformat[3]);

      if(0 == xioctl(VIDIOC_ENUM_FRAMESIZES, &v4l2.fsize[cnt])) {
        temp_stream<<v4l2.fsize[cnt].discrete.width;
        temp_stream<<"x";
        temp_stream<<v4l2.fsize[cnt].discrete.height;
        list->push_back(temp_stream.str());

      }
      else{
        break;
      }
    }
    memset(&v4l2.stream_fmt,0,sizeof(v4l2.stream_fmt));

    v4l2.stream_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if(0 == xioctl(VIDIOC_G_FMT, &v4l2.stream_fmt)) {
      temp_stream.str("");    // clearing temp_stream
      temp_stream<<v4l2.stream_fmt.fmt.pix.width;
      temp_stream<<"x";
      temp_stream<<v4l2.stream_fmt.fmt.pix.height;
      *str = temp_stream.str() ;
    }else{
      ROS_ERROR("failed");
    }
  }


  void Camera::enum_framerate(std::string pixelformat,int width,int height,
                              std::vector<std::string> *list,std::string *str)
  {

    std::stringstream temp_stream;
    for (int cnt=0 ;cnt< MAX_FRMINVAL_SUPPORT; cnt++){
      temp_stream.str("");    // clearing temp_stream
      memset(&v4l2.frminterval,0,sizeof(v4l2.frminterval));
      v4l2.frminterval.index	= cnt;
      v4l2.frminterval.width = width;
      v4l2.frminterval.height = height;
      v4l2.frminterval.pixel_format = v4l2_fourcc(pixelformat[0],
                                                  pixelformat[1],
                                                  pixelformat[2],
                                                  pixelformat[3]);

      if(0 == xioctl(VIDIOC_ENUM_FRAMEINTERVALS, &v4l2.frminterval)) {
        if(v4l2.frminterval.type == V4L2_FRMIVAL_TYPE_DISCRETE){
          v4l2.desc_frm_inval[cnt].numerator = v4l2.frminterval.discrete.numerator;
          v4l2.desc_frm_inval[cnt].denominator = v4l2.frminterval.discrete.denominator;
        }
        /*dividing denominator by numerator.
          for ex: (1) den-30, num- 1
                  30/1 = 30 FPS
                  (2) den-15, num -2
                  15/2 = 7.5 FPS
        */
        temp_stream<<(double)v4l2.desc_frm_inval[cnt].denominator/
                             v4l2.desc_frm_inval[cnt].numerator;
        temp_stream<<" FPS";
        list->push_back(temp_stream.str());
      }
      else{
        break;
      }
    }
  }
  /*****************************************************************************
   *  Name	:	set_format.
   *  Parameter1 : const std::string& fourcc - Name of the format,
   *                                           For ex: UYVY,MJPG
   *  Parameter2: int input_width- Width of the frame
   *  Parameter3: int input_height- Height of the frame
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function is to set the resolution and format.
   ****************************************************************************/
  int Camera::set_format(const std::string& fourcc,int input_width,int input_height)
  {
    memset(&v4l2.stream_fmt,0,sizeof(v4l2.stream_fmt));

    v4l2.stream_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.stream_fmt.fmt.pix.width = input_width;
    v4l2.stream_fmt.fmt.pix.height = input_height;
    v4l2.stream_fmt.fmt.pix.pixelformat = v4l2_fourcc(fourcc[0],
                                                      fourcc[1],
                                                      fourcc[2],
                                                      fourcc[3]);

    v4l2.stream_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if(ioctl(cam_fd, VIDIOC_S_FMT, &v4l2.stream_fmt)<0){
      ROS_INFO("VIDIOC_S_FMT failed");
      return FAILURE;
    }
    pixelformat = fourcc;
    height= input_height;
    width = input_width;
    return SUCCESS;
  }





  /*****************************************************************************
   *  Name	:	get_format.
   *  Parameter1 : const std::string * PixelFormat - Name of the format,
                                                     For ex: UYVY,MJPG
   *  Parameter2: int *height- Height of the frame
   *  Parameter3: int *width- Width of the frame
   *  Parameter4: int *Numerator- Numerator of framerate
   *  Parameter5: int *Denominator- Denominator of framerate
   *  Parameter6: int type - This denotes when the function is called.
                   The possible values are:
                   PIX_FORMAT                    1
                   RESOLUTION                    2
                   FPS                           3
   *  Description	: This function is to get the format,resolution and framerate.
   ****************************************************************************/
  void Camera::get_format(std::string *PixelFormat,int *Height,int *Width,
                          int *Numerator,int *Denominator,int type)
  {
    *PixelFormat=pixelformat;
    *Height = height;
    *Width = width;
    *Numerator = numerator;
    *Denominator = denominator;
    if(type==FPS){
      init=false;
    }
  }





  /*****************************************************************************
   *  Name	:	set_framerate.
   *  Parameter1: unsigned input_numerator - Numerator of the frame interval
   *  Parameter2: unsigned input_denominator - denominator of the frame interval
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function is to set the framerate.
   ****************************************************************************/
  int Camera::set_framerate(unsigned input_numerator, unsigned input_denominator)
  {
    memset(&v4l2.cam_parm,0,sizeof(v4l2.cam_parm));
    v4l2.cam_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(cam_fd, VIDIOC_G_PARM, &v4l2.cam_parm);
    if (!(v4l2.cam_parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)){
      ROS_WARN("V4L2_CAP_TIMEPERFRAME not supported by driver");
      ROS_WARN("Leaving frame rate unchanged");

      return FAILURE;
    }
    v4l2.cam_parm.parm.capture.timeperframe.numerator = input_numerator;
    v4l2.cam_parm.parm.capture.timeperframe.denominator = input_denominator;
    ioctl(cam_fd, VIDIOC_S_PARM, &v4l2.cam_parm);
    if (v4l2.cam_parm.parm.capture.timeperframe.numerator != input_numerator ||
      v4l2.cam_parm.parm.capture.timeperframe.denominator != input_denominator){
      ROS_WARN_STREAM("Driver rejected " <<input_numerator << "/"
                      << input_denominator << "s per frame and used "
                      << v4l2.cam_parm.parm.capture.timeperframe.numerator<< "/"
                      << v4l2.cam_parm.parm.capture.timeperframe.denominator
                      << "s instead");
    }
    numerator = v4l2.cam_parm.parm.capture.timeperframe.numerator;
    denominator = v4l2.cam_parm.parm.capture.timeperframe.denominator;
    return SUCCESS;
  }




  /*****************************************************************************
   *  Name	:	request_buffer.
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function will request buffer from the camera,
   *                  map the buffers and also enqueue the buffers.
   ****************************************************************************/
  int Camera::request_buffer()
  {
    memset(&v4l2.reqbuf,0,sizeof(v4l2.reqbuf));

    v4l2.reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.reqbuf.memory = V4L2_MEMORY_MMAP;
    v4l2.reqbuf.count = 2;

    if(ioctl(cam_fd, VIDIOC_REQBUFS, &v4l2.reqbuf)<0){
      ROS_INFO("VIDIOC_REQBUFS failed");
      return FAILURE;
    }

    buffers.resize(v4l2.reqbuf.count);
    for (unsigned i = 0; i < v4l2.reqbuf.count; i++){

      memset(&v4l2.buffer,0,sizeof(v4l2.buffer));


    v4l2.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.buffer.memory = V4L2_MEMORY_MMAP;
    v4l2.buffer.index = i;


    if(ioctl( cam_fd, VIDIOC_QUERYBUF, &v4l2.buffer) < 0) {
      ROS_INFO("VIDIOC_QUERYBUF failed");
      return FAILURE;
    }
    if(ioctl( cam_fd, VIDIOC_QBUF, &v4l2.buffer) < 0) {
      ROS_INFO("VIDIOC_QBUF failed");
      return FAILURE;
    }
    buffers[i].length = v4l2.buffer.length;
    buffers[i].start = NULL;
    buffers[i].start = static_cast<uint8_t *>
                       (mmap(NULL, v4l2.buffer.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, cam_fd, v4l2.buffer.m.offset));

    if (NULL == buffers[i].start)
      ROS_INFO("MAP_FAILED");
    }
    return SUCCESS;
  }




  /*****************************************************************************
   *  Name	:	stream_on.
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function will stream on the camera.
   ****************************************************************************/
  int Camera::stream_on()
  {
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl( cam_fd, VIDIOC_STREAMON, &type) < 0) {
      ROS_INFO("VIDIOC_STREAMON failed");
      return FAILURE;
    }else{
      streamon=true;
    }
    return SUCCESS;
  }




  /****************************************************************************
   *  Name	:	enqueue.
   *  Parameter1: int index- The buffer index in which frame will be stored.
   *  Description	:   This function will enqueue the buffer
   ****************************************************************************/
  void Camera::enqueue(int index)
  {
    memset(&v4l2.buffer,0,sizeof(v4l2.buffer));
    v4l2.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.buffer.memory = V4L2_MEMORY_MMAP;
    v4l2.buffer.index = index;
    ioctl(cam_fd, VIDIOC_QBUF, &v4l2.buffer);
  }




  /*****************************************************************************
   *  Name	:	capture.
   *  Parameter1: ecam_v4l2::image *image-The image in which the frame is stored.
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *        DEV_UNPLUGGED - If the device is unplugged.
   *  Description	: This function will dequeue the buffer and create image data.
   ****************************************************************************/
  int Camera::capture(ecam_v4l2::image *image)
  {
    memset(&v4l2.buffer,0,sizeof(v4l2.buffer));
    v4l2.buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.buffer.memory = V4L2_MEMORY_MMAP;
    if(ioctl(cam_fd, VIDIOC_DQBUF, &v4l2.buffer)<0){
      if(errno==ENODEV){
        image->format = "unplugged";
        image->data.reserve(1);
        image->data.resize(1);
        ROS_WARN("Device Unplugged");
        return DEV_UNPLUGGED;
      }
      return FAILURE;
    }
    if (v4l2.buffer.flags & V4L2_BUF_FLAG_ERROR){
      enqueue(v4l2.buffer.index);
      ROS_WARN("Camera driver notified buffer error, ignoring frame");
      return FAILURE;
    }else{
      image->width = v4l2.stream_fmt.fmt.pix.width;
      image->height = v4l2.stream_fmt.fmt.pix.height;
      image->length = v4l2.buffer.bytesused;
      if(v4l2.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_MJPEG){

        image->format = "mjpg";
        image->data.reserve(v4l2.stream_fmt.fmt.pix.height *
                            v4l2.stream_fmt.fmt.pix.width);
        image->data.resize(v4l2.stream_fmt.fmt.pix.width *
                           v4l2.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[v4l2.buffer.index].start,
               v4l2.stream_fmt.fmt.pix.width *
               v4l2.stream_fmt.fmt.pix.height);

      }else if(v4l2.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_GREY){
        if(v4l2.buffer.bytesused!=v4l2.stream_fmt.fmt.pix.height *
                                  v4l2.stream_fmt.fmt.pix.width)
        {
          enqueue(v4l2.buffer.index);
          return FAILURE;
        }
        image->format = "mono8";
        image->data.reserve(v4l2.stream_fmt.fmt.pix.height *
                            v4l2.stream_fmt.fmt.pix.width);
        image->data.resize(v4l2.stream_fmt.fmt.pix.width *
                           v4l2.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[v4l2.buffer.index].start,
               v4l2.stream_fmt.fmt.pix.width *
               v4l2.stream_fmt.fmt.pix.height);

      }else if(v4l2.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_Y16){
        if(v4l2.buffer.bytesused!=v4l2.stream_fmt.fmt.pix.height *
                                  v4l2.stream_fmt.fmt.pix.width  * 2)
        {
          enqueue(v4l2.buffer.index);
          return FAILURE;
        }
        image->format = "mono16";
        image->data.reserve(2* v4l2.stream_fmt.fmt.pix.height *
                               v4l2.stream_fmt.fmt.pix.width);
        image->data.resize(2 * v4l2.stream_fmt.fmt.pix.width *
                               v4l2.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[v4l2.buffer.index].start,
               2*v4l2.stream_fmt.fmt.pix.width *
                 v4l2.stream_fmt.fmt.pix.height);

      }else if(v4l2.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_UYVY){
        if(v4l2.buffer.bytesused!=v4l2.stream_fmt.fmt.pix.height *
                                  v4l2.stream_fmt.fmt.pix.width  * 2)
        {
          enqueue(v4l2.buffer.index);
          return FAILURE;
        }
        image->format = "uyvy";
        image->data.reserve(2 * v4l2.stream_fmt.fmt.pix.height *
                                v4l2.stream_fmt.fmt.pix.width);
        image->data.resize(2 * v4l2.stream_fmt.fmt.pix.width *
                               v4l2.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[v4l2.buffer.index].start,
               2*v4l2.stream_fmt.fmt.pix.width *
                 v4l2.stream_fmt.fmt.pix.height);

      }else if( v4l2.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_YUYV){
        if(v4l2.buffer.bytesused!=v4l2.stream_fmt.fmt.pix.height *
                                  v4l2.stream_fmt.fmt.pix.width  * 2)
        {
          enqueue(v4l2.buffer.index);
          return FAILURE;
        }
        image->format = "yuyv";
        image->data.reserve(2 * v4l2.stream_fmt.fmt.pix.height *
                                v4l2.stream_fmt.fmt.pix.width);
        image->data.resize(2 * v4l2.stream_fmt.fmt.pix.width *
                               v4l2.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[v4l2.buffer.index].start,
               2*v4l2.stream_fmt.fmt.pix.width *
                 v4l2.stream_fmt.fmt.pix.height);

      }else if(v4l2.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_Y12){
        if(v4l2.buffer.bytesused!=v4l2.stream_fmt.fmt.pix.height *
                                  v4l2.stream_fmt.fmt.pix.width  * 1.5)
        {
          enqueue(v4l2.buffer.index);
          return FAILURE;
        }
        image->format = "mono12";
        image->data.reserve(1.5 * v4l2.stream_fmt.fmt.pix.height *
                                  v4l2.stream_fmt.fmt.pix.width);
        image->data.resize(1.5 * v4l2.stream_fmt.fmt.pix.width *
                                 v4l2.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[v4l2.buffer.index].start,
               1.5*v4l2.stream_fmt.fmt.pix.width *
                   v4l2.stream_fmt.fmt.pix.height);

      }else if(v4l2.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_SBGGR8){
        if(v4l2.buffer.bytesused!=v4l2.stream_fmt.fmt.pix.height *
                                  v4l2.stream_fmt.fmt.pix.width)
        {
          enqueue(v4l2.buffer.index);
          return FAILURE;
        }
        image->format = "ba81";
        image->data.reserve(v4l2.stream_fmt.fmt.pix.height *
                            v4l2.stream_fmt.fmt.pix.width);
        image->data.resize(v4l2.stream_fmt.fmt.pix.width *
                           v4l2.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[v4l2.buffer.index].start,
               v4l2.stream_fmt.fmt.pix.width *
               v4l2.stream_fmt.fmt.pix.height);
      }
    }
    enqueue(v4l2.buffer.index);
    return SUCCESS;
  }






  /*****************************************************************************
   *  Name	:	stream_off.
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function will stream off the camera.
   ****************************************************************************/
  int Camera::stream_off()
  {
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl( cam_fd, VIDIOC_STREAMOFF, &type) < 0) {
      ROS_INFO("VIDIOC_STREAMOFF failed");
      return FAILURE;
    }else{
      streamon=false;
    }
    return SUCCESS;
  }




  /*****************************************************************************
  *  Name	:	clean_buffer.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function will clean the buffer in the camera.
  *****************************************************************************/
  int Camera::clean_buffer()
  {
    munmap_buffers();
    memset(&v4l2.reqbuf,0,sizeof(v4l2.reqbuf));
    v4l2.reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2.reqbuf.memory = V4L2_MEMORY_MMAP;
    v4l2.reqbuf.count = 0;
    if(ioctl( cam_fd, VIDIOC_REQBUFS, &v4l2.reqbuf) < 0) {
      ROS_INFO("VIDIOC_REQBUFS failed");
      return FAILURE;
    }else{
      return SUCCESS;
    }
  }

  /*****************************************************************************
  *  Name	:	munmap_buffers.
  *  Description	:   This function will unmaap the allocated buffer.
  *****************************************************************************/
  void Camera::munmap_buffers()
  {
      int index;
      if(buffers.size()!=0) {
          for (index = 0; index < v4l2.reqbuf.count; index++) {
              if(buffers[index].start!=NULL){
                  if(munmap(buffers[index].start, buffers[index].length)==SUCCESS){
                    buffers[index].start = NULL;
                  }
              }
          }
      }
  }


  /*****************************************************************************
   *  Name	:	close_device.
   *  Description	:   This function is to close the camera file descriptor
   ****************************************************************************/
  void Camera::close_device()
  {
    // checking if camera is already opened
    if(cam_fd>-1){
      close(cam_fd);
      cam_fd=-1;
    }
  }




  /*****************************************************************************
  *  Name	:	onInit.
  *  Returns	:
  *  			True	- If the camera is initialized
  *       False - if the camera is not initialized
  *  Description	: This function is to check whether camera
  *                 is already initialized or not.
  *****************************************************************************/
  bool Camera::onInit()
  {
    return init;
  }




  /*****************************************************************************
  *  Name	:	set_init.
  *  Description	:   This function is to set the init value
  *                   and initialize subscribers count.
  *****************************************************************************/
  void Camera::set_init(bool value)
  {
    init=value;
    if(subscribers_count<0){
      subscribers_count = 0;
    }
    subscribers_count++;
  }





  /*****************************************************************************
  *  Name	:	sub_count.
  *  Returns	: int	- number of subscribers.
  *  Description	:   This function is to get the subscribers count.
  *****************************************************************************/
  int Camera::sub_count()
  {
    subscribers_count--;
    return subscribers_count;
  }





  /*****************************************************************************
  *  Name	:	isStreamOn.
  *  Returns	:
  *  			True	- If the camera is streamed on
  *       False - if the camera is not streamed off.
  *  Description:This function is to check whether camera is streamed on or off.
  *****************************************************************************/
  bool Camera::isStreamOn()
  {
    return streamon;
  }




  /*****************************************************************************
   *  Name	:	set_camera_name.
   *  Parameter1: std::string name - Name of the camera.
   *  Parameter2 : std::string node_name - Node name of the camera. (Ex: video0)
   *  Description	:   This function is to set the camera name and the node name.
   ****************************************************************************/
  void Camera::set_camera_name(std::string name, std::string node_name)
  {
    camera_name = name;
    video_node = node_name;
  }




  /*****************************************************************************
   *  Name	:	get_camera_name.
   *  Returns	: std::string - Name of the camera.
   *  Description	:   This function is to get the camera name.
   ****************************************************************************/
  std::string Camera::get_camera_name()
  {
    return camera_name;
  }




  /*****************************************************************************
   *  Name	:	get_camera_name.
   *  Returns	: std::string - Node name of the camera.
   *  Description	:   This function is to get the video node name.
   ****************************************************************************/
  void Camera::get_node_name(std::string *dev_node_name)
  {
    *dev_node_name += video_node;
  }





  /*****************************************************************************
  *  Name	:	xioctl.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			errno	- Failed to perform specified operation
  *  Description	:   This function is to make the ioctl call.
  *****************************************************************************/
  int Camera::xioctl(unsigned cmd, void *arg)
  {
    if(ioctl(cam_fd,cmd,arg)<0){
      return errno;
    }
    return SUCCESS;
  }





  /*****************************************************************************
   *  Name	:	get_four_character_code.
   *  Parameter1: int32_t pix_fmt - pixel_format value.
   *  Parameter2 : std::string *pixelformat - Name of the format,
   *                                          For ex: UYVY,MJPG
   *  Description	:   This function is to get the four character code as
   *                  computed by the v4l2_fourcc() macro.
   ****************************************************************************/
  void Camera::get_four_character_code(int32_t pix_fmt,std::string *pixelformat)
  {
    switch (pix_fmt) {
      case V4L2_PIX_FMT_MJPEG:
        *pixelformat = "MJPG";
        break;
      case V4L2_PIX_FMT_UYVY:
        *pixelformat = "UYVY";
        break;
      case V4L2_PIX_FMT_YUYV:
        *pixelformat = "YUYV";
        break;
      case V4L2_PIX_FMT_GREY:
        *pixelformat = "GREY";
        break;
      case V4L2_PIX_FMT_Y16:
        *pixelformat = "Y16 ";
        break;
      case V4L2_PIX_FMT_Y12:
        *pixelformat = "Y12 ";
        break;
      case V4L2_PIX_FMT_SBGGR8:
        *pixelformat = "BA81";
      default:
        break;
    }
  }


} //namespace ecam_v4l2
