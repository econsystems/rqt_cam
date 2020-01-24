#include "ecam_v4l2/V4l2.h"

namespace ecam_v4l2
{


  // Constructor of class Camera
  Camera::Camera()
  {
    streamon=false;
    cam_fd=-1;
    init=true;
  }





  /************************************************************************************************************
   *  Name	:	open_device.
   *  Parameter1 : const std::string &device - Name of the device node, For ex. /dev/video0
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function is to open the camera file descriptor.
   ************************************************************************************************************/
  int Camera::open_device(const std::string &device)
  {
    struct v4l2_capability cap;
    // checking if camera is not already open.
    if(cam_fd==-1){
      cam_fd=open(device.c_str(),O_RDWR);
    }
    // if open camera failed cam_fd will be equal to -1
    if(cam_fd<0){
      ROS_INFO("Open device Failed");
      return FAILURE;
    }

    ioctl(cam_fd, VIDIOC_QUERYCAP, &cap);

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
      ROS_INFO("Device does not support video capture");
      return FAILURE;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)){
      ROS_INFO("Device does not support streaming I/O");
      return FAILURE;
    }
    return SUCCESS;
  }




  /************************************************************************************************************
   *  Name	:	init_format.
   *  Description	:   This function is to setup the camera to streamon, once it is selected.
   ************************************************************************************************************/
  void Camera::init_format()
  {
    int32_t pix_fmt;

    frmt.ffmt[0].index		= 0;
    frmt.ffmt[0].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(0 == ioctl(cam_fd,VIDIOC_ENUM_FMT, &frmt.ffmt[0])) {
      pix_fmt = frmt.ffmt[0].pixelformat;
    }
    get_four_character_code(pix_fmt,&pixelformat);
    frmt.fsize[0].index		= 0;
    frmt.fsize[0].pixel_format	= pix_fmt;	

    if(0 == ioctl(cam_fd,VIDIOC_ENUM_FRAMESIZES, &frmt.fsize[0])) {
      width = frmt.fsize[0].discrete.width;
      height = frmt.fsize[0].discrete.height;
    }
    frmt.frminterval.index	= 0;
    frmt.frminterval.width = width;
    frmt.frminterval.height = height;
    frmt.frminterval.pixel_format = pix_fmt;

    if(0 == ioctl(cam_fd,VIDIOC_ENUM_FRAMEINTERVALS, &frmt.frminterval)) {
      if(frmt.frminterval.type == V4L2_FRMIVAL_TYPE_DISCRETE){
        numerator = frmt.frminterval.discrete.numerator;
        denominator = frmt.frminterval.discrete.denominator;
      }
    }

    set_format(pixelformat,width,height);
    set_framerate(numerator, denominator);
    request_buffer();
    stream_on();
    init=true;
  }




  /************************************************************************************************************
   *  Name	:	set_format.
   *  Parameter1 : const std::string& fourcc - Name of the format, For ex: UYVY,MJPG
   *  Parameter2: int width- Width of the frame
   *  Parameter3: int height- Height of the frame
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function is to set the resolution and format.
   ************************************************************************************************************/
  int Camera::set_format(const std::string& fourcc,int width,int height)
  {
    struct v4l2_format format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
    format.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if(ioctl(cam_fd, VIDIOC_S_FMT, &format)<0){
      ROS_INFO("VIDIOC_S_FMT failed");
      return FAILURE;
    }
    frmt.stream_fmt.fmt.pix.pixelformat = v4l2_fourcc(fourcc[0],fourcc[1],fourcc[2],fourcc[3]);
    frmt.stream_fmt.fmt.pix.width = width;
    frmt.stream_fmt.fmt.pix.height = height;
    return SUCCESS;
  }






  /************************************************************************************************************
   *  Name	:	get_format.
   *  Parameter1 : const std::string * PixelFormat - Name of the format, For ex: UYVY,MJPG
   *  Parameter2: int width- Width of the frame
   *  Parameter3: int height- Height of the frame
   *  Description	:   This function is to get the resolution and format.
   ************************************************************************************************************/
  void Camera::get_format(std::string *PixelFormat,int *Height,int *Width,int type)
  {
    *PixelFormat=pixelformat;
    *Height = height;
    *Width = width;
    if(type==FPS){
      init=false;
    }
  }





  /************************************************************************************************************
   *  Name	:	set_framerate.
   *  Parameter1: unsigned numerator - Numerator of the frame interval
   *  Parameter2: unsigned denominator - denominator of the frame interval
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function is to set the framerate.
   ************************************************************************************************************/
  int Camera::set_framerate(unsigned numerator, unsigned denominator)
  {
    struct v4l2_streamparm streamparm = {};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(cam_fd, VIDIOC_G_PARM, &streamparm);
    if (!(streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)){
      ROS_WARN("V4L2_CAP_TIMEPERFRAME not supported by driver, leaving frame rate unchanged");
      return FAILURE;
    }
    streamparm.parm.capture.timeperframe.numerator = numerator;
    streamparm.parm.capture.timeperframe.denominator = denominator;
    ioctl(cam_fd, VIDIOC_S_PARM, &streamparm);
    if (streamparm.parm.capture.timeperframe.numerator != numerator ||
      streamparm.parm.capture.timeperframe.denominator != denominator){
      ROS_WARN_STREAM("Driver rejected " <<numerator << "/" << denominator << "s per frame and used "
      << streamparm.parm.capture.timeperframe.numerator << "/"
      << streamparm.parm.capture.timeperframe.denominator << "s instead");
    }
    return SUCCESS;
  }




  /************************************************************************************************************
   *  Name	:	request_buffer.
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function will request buffer from the camera and map the buffers.
   *                  and also enqueue the buffers.
   ************************************************************************************************************/
  int Camera::request_buffer()
  {
    struct v4l2_requestbuffers reqbuf = {};
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = 2;

    if(ioctl(cam_fd, VIDIOC_REQBUFS, &reqbuf)<0){
      ROS_INFO("VIDIOC_REQBUFS failed");
      return FAILURE;
    }

    buffers.resize(reqbuf.count);
    for (unsigned i = 0; i < reqbuf.count; i++){
    struct v4l2_buffer buffer = {};

    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;


    if(ioctl( cam_fd, VIDIOC_QUERYBUF, &buffer) < 0) {
      ROS_INFO("VIDIOC_QUERYBUF failed");
      return FAILURE;
    }
    if(ioctl( cam_fd, VIDIOC_QBUF, &buffer) < 0) {
      ROS_INFO("VIDIOC_QBUF failed");
      return FAILURE;
    }
    buffers[i].length = buffer.length;
    buffers[i].start = static_cast<uint8_t *>(mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
    MAP_SHARED, cam_fd, buffer.m.offset));

    if (MAP_FAILED == buffers[i].start)
      ROS_INFO("MAP_FAILED");
    }
    return SUCCESS;
  }




  /************************************************************************************************************
   *  Name	:	stream_on.
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function will stream on the camera.
   ************************************************************************************************************/
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




  /************************************************************************************************************
   *  Name	:	enqueue.
   *  Parameter1: int index- The buffer index in which frame will be stored.
   *  Description	:   This function will enqueue the buffer
   ************************************************************************************************************/
  void Camera::enqueue(int index)
  {
    struct v4l2_buffer buffer = {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    ioctl(cam_fd, VIDIOC_QBUF, &buffer);
  }




  /************************************************************************************************************
   *  Name	:	capture.
   *  Parameter1: int *got_frame- Flag to indicate where the frame is a proper frame.
   *  Returns	:	const boost::shared_ptr<Camera::V4L2Image> image - The image in which the frame will be stored.
   *  Description	:   This function will dequeue the buffer and create image data.
   ************************************************************************************************************/
  int Camera::capture(ecam_v4l2::image *image)
  {
    struct v4l2_buffer buffer = {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    ioctl(cam_fd, VIDIOC_DQBUF, &buffer);
    if (buffer.flags & V4L2_BUF_FLAG_ERROR){
      enqueue(buffer.index);
      ROS_WARN("Camera driver notified buffer error, ignoring frame");
      return FAILURE;
    }else{

      if(empty_image(buffer.bytesused,image)<0){
        return FAILURE;
      }
      if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_MJPEG){

        image->data.resize(frmt.stream_fmt.fmt.pix.width * frmt.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[buffer.index].start,frmt.stream_fmt.fmt.pix.width*frmt.stream_fmt.fmt.pix.height);

      }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_GREY){

        image->data.resize(frmt.stream_fmt.fmt.pix.width * frmt.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[buffer.index].start,frmt.stream_fmt.fmt.pix.width*frmt.stream_fmt.fmt.pix.height);

      }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_Y16){

        image->data.resize(2 * frmt.stream_fmt.fmt.pix.width * frmt.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[buffer.index].start,2*frmt.stream_fmt.fmt.pix.width*frmt.stream_fmt.fmt.pix.height);

      }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_UYVY ||frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_YUYV){

        image->data.resize(2 * frmt.stream_fmt.fmt.pix.width * frmt.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[buffer.index].start,2*frmt.stream_fmt.fmt.pix.width*frmt.stream_fmt.fmt.pix.height);

      }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_Y12){

        image->data.resize(1.5 * frmt.stream_fmt.fmt.pix.width * frmt.stream_fmt.fmt.pix.height);
        memcpy(&image->data[0],buffers[buffer.index].start,1.5*frmt.stream_fmt.fmt.pix.width*frmt.stream_fmt.fmt.pix.height);

      }
    }
    enqueue(buffer.index);
    return SUCCESS;
  }





  /************************************************************************************************************
   *  Name	:	empty_image.
   *  Parameter1: int buffer_index- The buffer index in which frame is stored.
   *  Returns	:	const boost::shared_ptr<Camera::V4L2Image> image - The image in which the frame will be stored.
   *  Description	:   This function will reserve space for image data.
   ************************************************************************************************************/
  int Camera::empty_image(int buffer_len,ecam_v4l2::image *image)
  {
    image->width = frmt.stream_fmt.fmt.pix.width;
    image->height = frmt.stream_fmt.fmt.pix.height;
    image->length = buffer_len;
    if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_MJPEG){

      image->format = "mjpg";
      image->data.reserve(frmt.stream_fmt.fmt.pix.height * frmt.stream_fmt.fmt.pix.width);

    }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_GREY){

      image->format = "mono8";
      image->data.reserve(frmt.stream_fmt.fmt.pix.height * frmt.stream_fmt.fmt.pix.width);

    }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_Y16){

      image->format = "mono16";
      image->data.reserve(2* frmt.stream_fmt.fmt.pix.height * frmt.stream_fmt.fmt.pix.width);

    }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_UYVY){

      image->format = "uyvy";
      image->data.reserve(2 * frmt.stream_fmt.fmt.pix.height * frmt.stream_fmt.fmt.pix.width);

    }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_YUYV){

      image->format = "yuyv";
      image->data.reserve(2 * frmt.stream_fmt.fmt.pix.height * frmt.stream_fmt.fmt.pix.width);

    }else if(frmt.stream_fmt.fmt.pix.pixelformat==V4L2_PIX_FMT_Y12){

      image->format = "mono12";
      image->data.reserve(1.5 * frmt.stream_fmt.fmt.pix.height * frmt.stream_fmt.fmt.pix.width);

    }
    return SUCCESS;
  }





  /************************************************************************************************************
   *  Name	:	stream_off.
   *  Returns	:	Function result depends on return value of child functions and
   *  			condition available in the functions based on this return value will
   *
   *  			SUCCESS	- all the condition executed properly
   *  			FAILURE	- Failed to perform specified operation
   *  Description	:   This function will stream off the camera.
   ************************************************************************************************************/
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




  /************************************************************************************************************
  *  Name	:	clean_buffer.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			FAILURE	- Failed to perform specified operation
  *  Description	:   This function will clean the buffer in the camera.
  ************************************************************************************************************/
  int Camera::clean_buffer()
  {
    struct v4l2_requestbuffers reqbuf = {};
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = 0;
    for (auto &x : buffers)
    munmap(x.start, x.length);

    if(ioctl( cam_fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
      ROS_INFO("VIDIOC_REQBUFS failed");
      return FAILURE;
    }else{
      return SUCCESS;
    }
  }





  /************************************************************************************************************
   *  Name	:	close_device.
   *  Description	:   This function is to close the camera file descriptor
   ************************************************************************************************************/
  void Camera::close_device()
  {
    // checking if camera is already opened
    if(cam_fd>-1){
      close(cam_fd);
      cam_fd=-1;
    }
  }




  /************************************************************************************************************
  *  Name	:	onInit.
  *  Returns	:
  *  			True	- If the camera is initialized
  *       False - if the camera is not initialized
  *  Description	:   This function is to check whether camera is already initialized or not.
  ************************************************************************************************************/
  bool Camera::onInit()
  {
    return init;
  }




  /************************************************************************************************************
  *  Name	:	isStreamOn.
  *  Returns	:
  *  			True	- If the camera is streamed on
  *       False - if the camera is not streamed off.
  *  Description	:   This function is to check whether camera is streamed on or off.
  ************************************************************************************************************/
  bool Camera::isStreamOn()
  {
    return streamon;
  }








  /************************************************************************************************************
  *  Name	:	xioctl.
  *  Returns	:	Function result depends on return value of child functions and
  *  			condition available in the functions based on this return value will
  *
  *  			SUCCESS	- all the condition executed properly
  *  			errno	- Failed to perform specified operation
  *  Description	:   This function is to make the ioctl call.
  ************************************************************************************************************/
  int Camera::xioctl(unsigned cmd, void *arg)
  {
    if(ioctl(cam_fd,cmd,arg)<0){
      return errno;
    }
    return SUCCESS;
  }





  /************************************************************************************************************
   *  Name	:	get_four_character_code.
   *  Parameter1: int32_t pix_fmt - pixel_format value.
   *  Parameter2 : std::string *pixelformat - Name of the format, For ex: UYVY,MJPG
   *  Description	:   This function is to get the four character code as computed by the v4l2_fourcc() macro.
   ************************************************************************************************************/
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
      default:
        break;
    }
  }


} //namespace ecam_v4l2
