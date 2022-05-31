

# rqt_cam : Open Source ROS V4L2 Camera Plugin

High performance video capture using Video4Linux2. This package consists of a publisher node (ecam_v4l2) and an image viewer plugin(rqt_cam).

## ecam_v4l2

"ecam_v4l2" is the publisher node which publishes image in the topic corresponding to the name of the camera.
It also provides Services for Selecting Cameras, changing V4l2 controls and switching Color Space/Compression, Resolution and Framerate.

### Published Topics
The Number of topics published is based on the number of cameras connected.
The topic name will be the name of the camera.
For example,
* /ecam_v4l2/See3CAM_CU20_C7500000

### List of Services:
* /ChooseDevice
* /QueryControl
* /SetControl
* /EnumerateFormat
* /Setformat

## rqt_cam

"rqt_cam" is an image viewer plugin which will get all the topics published of type "ecam_v4l2/image" and will stream the image data.

It also provides UI for changing V4l2 controls and to switch Color space/Compression, Resolution and Framerate.

### Supported v4l2 controls:
* Brightness
* Contrast
* Saturation
* Sharpness
* Gamma
* Gain
* Hue
* White Balance
* Zoom
* Pan
* Tilt
* Exposure
* Focus (Absolute)
* Focus
* Backlight Compensation

### Supported Color space/Compression:

* UYVY
* YUYV
* MJPG
* Y8
* Y12
* Y16

### Supported E-con Camera's
#### USB Camera

	* See3CAM_CU20
	* See3CAM_CU30
	* See3CAM_CU38
	* See3CAMCU50
	* See3CAM_CU51
	* See3CAM_CU130
	* See3CAM_CU135
	* See3CAM_11CUG
	* See3CAM_12CUNIR
	* See3CAM_30
	* See3CAM_81
	* See3CAM_130 Autofocus
	* See3CAM_CU55M
	* See3CAM_20CUG
  * See3CAM_CU135M_H01R1
  * See3CAM_CU135M_H03R1
#### MIPI camera
	* e-CAM130_CUXVR


## Prerequisites

- For Ubuntu 16.04 [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- For Ubuntu 18.04 [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

## How to Use

- Build the package using [this Build Manual](https://github.com/econsystems/rqt_cam/tree/master/documents)

- Run the rqt_cam application using [this User Manual](https://github.com/econsystems/rqt_cam/tree/master/documents)


## Releases

* Latest releases can be downloaded from [this link](https://github.com/econsystems/rqt_cam/releases)

## Release

* rqt_cam v1.0.0		-	20-Jan-2020

## Support

If you need assistance, visit at https://www.e-consystems.com/create-ticket.asp or contact us at techsupport@e-consystems.com
