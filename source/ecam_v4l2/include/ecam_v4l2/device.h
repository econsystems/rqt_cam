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
#ifndef DEVICE_H
#define DEVICE_H
#include <libudev.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <ros/ros.h>
#define USB 1
#define MIPI 2
#define FAILURE                 -1
#define SUCCESS                  0
#define V4L2_CAP_META_CAPTURE    0x00800000


namespace ecam_v4l2
{

  class Devices
  {
  private:
    std::vector<std::string>device_node_name;
    std::vector<std::string>camera_name;
    bool check_for_valid_videonode(std::string &dev_node);
    int check_camera_type(std::string &dev_node,
                          std::string *productName,int *type);
    bool get_product_name(std::string &dev_node);
    bool isEnumerated(std::string node_name);
    void prepare_topic(std::string *productName);
  public:
    int list_devices();
    int get_camera_count();
    bool remove_camera(std::string cam_name);
    bool get_camera(int index,std::string *cam_name,std::string *dev_node_name);
    std::vector<std::string> get_list();
  };
} // namespace ecam_v4l2


#endif
