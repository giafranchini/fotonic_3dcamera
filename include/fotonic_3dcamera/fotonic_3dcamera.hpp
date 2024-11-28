/*
 * Software License Agreement (BSD License)
 *
 *  fotonic_3dcamera
 *  Copyright (c) 2012, Robotnik Automation, SLL
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <memory>
#include <thread>
#include <stdio.h>

// #include "self_test/self_test.h"
// #include "diagnostic_msgs/DiagnosticStatus.h"
// #include "diagnostic_updater/diagnostic_updater.h"
// #include "diagnostic_updater/update_functions.h"
// #include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fotonic_3dcamera/camera.h"

#define FOTONIC_DESIRED_FREQ    10.0

namespace fotonic
{

class Camera3D : public rclcpp::Node
{
public:
  explicit Camera3D(const rclcpp::NodeOptions & options);

  ~Camera3D();

  void InitSensor();
  int start();
  int stop();
  bool PreZFilterPixel(short * pPixels, int iNumX, int iNumY, int x, int y);
  bool NoiseFilterPixel(short * pPixels, int iNumX, int iNumY, int x, int y);
  void getData();
  int read_and_publish();
	bool loop();

private:
  // Config params
  std::string ip_address_;
  std::string base_frame_id_;
  std::string fotonic_frame_id_;
  std::string name_;
  int startup_delay_;

  bool running;

  // Error counters and flags
  int error_count_ = 0;
  int slow_count_ = 0;
  std::string was_slow_;
  std::string error_status_;

  double desired_freq_ = 10.0;
  // diagnostic_updater::FrequencyStatus freq_diag_;

  // Camera and filter variables
  C_FZCamera * camera_;

  // Sensor parameters
  double sensor_shutter_ms_;
  int sensor_framerate_;
  int sensor_framerate_divisor_;

  // Hardware filters
  bool lerp_filter_enable_;         // true
  bool edge_filter_enable_;         // false
  int edge_filter_minB_;            // 1
  int edge_filter_diff1_;           // 10
  int edge_filter_diff2_;           // 10
  int edge_filter_diff3_;           // 20

  // Software filters
  int active_brightness_min_;           // 20
  int active_brightness_max_;           // 4095
  int z_range_min_;                 // 0
  int z_range_max_;                 // 7000
  bool pre_zfilterx_enable_;            // false
  bool pre_zfiltery_enable_;            // false
  int pre_zfilter_depth_;               // 150
  bool noise_filter_enable_;            // false
  int noise_filter_radius_;
  int noise_filter_depth_;
  int noise_filter_pixels_;

  float max_delay;

	std::thread t;

  // TODO(giafranchini): port this
  // self_test::TestRunner self_test_;
  // diagnostic_updater::Updater diagnostic_;
  // TODO(giafranchini): port this

  rclcpp::Node::SharedPtr node_handle_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fotonic_3Ddata_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_depth_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_depth_pub_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  image_transport::ImageTransport it;
  image_transport::Publisher pub_;
  image_transport::Publisher pub_depth_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_fotonic_;

};
} // namespace fotonic
