/*
 * Software License Agreement (BSD License)
 *
 *  fotonic_3dcamera_node
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

#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/utilities.hpp>

#include "fotonic_3dcamera/fotonic_3dcamera.hpp"

namespace fotonic
{
// TODO(giafranchini): maybe make this a parameter?
#define FOTONIC_DESIRED_FREQ    10.0

Camera3D::Camera3D(const rclcpp::NodeOptions & options)
: Node("fotonic_3dcamera", options),
  node_handle_(std::shared_ptr<Camera3D>(this, [](auto *) {})),
  it(node_handle_)
{
  running = false;

  // read node parameters
  startup_delay_ = declare_parameter<int>("startup_delay", 0);
  ip_address_ = declare_parameter<std::string>("ip_address", "192.168.1.10");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  fotonic_frame_id_ = declare_parameter<std::string>("fotonic_frame_id", "fotonic_link_optical");

  // Sensor parameters
  sensor_shutter_ms_ = declare_parameter<double>("sensor_shutter_ms", 10.0);
  sensor_framerate_ = declare_parameter<int>("sensor_framerate", 40);
  sensor_framerate_divisor_ = declare_parameter<int>("sensor_framerate_divisor", 1);

  // Hardware filters
  lerp_filter_enable_ = declare_parameter<bool>("lerp_filter_enable", true);
  edge_filter_enable_ = declare_parameter<bool>("edge_filter_enable", false);
  edge_filter_minB_ = declare_parameter<int>("edge_filter_minB", 1);
  edge_filter_diff1_ = declare_parameter<int>("edge_filter_diff1", 10);
  edge_filter_diff2_ = declare_parameter<int>("edge_filter_diff2", 10);
  edge_filter_diff3_ = declare_parameter<int>("edge_filter_diff3", 20);
  // Software filters
  active_brightness_min_ = declare_parameter<int>("active_brightness_min", 20);
  active_brightness_max_ = declare_parameter<int>("active_brightness_max", 4095);
  z_range_min_ = declare_parameter<int>("z_range_min", 0);
  z_range_max_ = declare_parameter<int>("z_range_max", 7000);
  pre_zfilterx_enable_ = declare_parameter<bool>("pre_zfilterx_enable", false);
  pre_zfiltery_enable_ = declare_parameter<bool>("pre_zfiltery_enable", false);
  pre_zfilter_depth_ = declare_parameter<int>("pre_zfilter_depth", 150);
  noise_filter_enable_ = declare_parameter<bool>("noise_filter_enable", false);
  noise_filter_radius_ = declare_parameter<int>("noise_filter_radius", 1);
  noise_filter_depth_ = declare_parameter<int>("noise_filter_depth", 10);
  noise_filter_pixels_ = declare_parameter<int>("noise_filter_pixels", 20);

  RCLCPP_INFO(get_logger(), "Initializing Fotonic 3D Camera component");
  RCLCPP_INFO(get_logger(), "Brightness [%d - %d]", active_brightness_min_, active_brightness_max_);
  RCLCPP_INFO(get_logger(), "Z Range: [%d - %d]", z_range_min_, z_range_max_);
  RCLCPP_INFO(
    get_logger(), "Pre Z Filter [X]: %d Pre Z Filter [Y]:%d", pre_zfilterx_enable_,
    pre_zfiltery_enable_);
  RCLCPP_INFO(get_logger(), "Noise Filter: %d", noise_filter_enable_);
  RCLCPP_INFO(get_logger(), "Frame: %s", fotonic_frame_id_.c_str());

  // TODO(giafranchini): atm we leave the publishers name as they are, then we could think about uniform them
  fotonic_3Ddata_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 1);
  // image_pub_ = create_publisher<sensor_msgs::msg::Image>("image", 1);
  // image_depth_pub_ = create_publisher<sensor_msgs::msg::Image>("image_depth", 1);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("ir/camera_info", 1);
  camera_info_depth_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 1);
  temperature_pub_ = create_publisher<std_msgs::msg::Float32>("temperature", 1);

  // it = image_transport::ImageTransport(this);
  pub_ = it.advertise("ir/image_raw", 1);
  pub_depth_ = it.advertise("depth/image_raw", 1);

  camera_info_manager_fotonic_ =
    std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "fotonic_3dcamera", "package://fotonic_3dcamera/fotonic_calibration.yaml");

  // TODO(giafranchini): maybe make this a parameter?
  camera_info_manager_fotonic_->loadCameraInfo("package://fotonic_3dcamera/params/fotonic_calibration.yaml");
  camera_info_msg_ = camera_info_manager_fotonic_->getCameraInfo();

  // TODO(giafranchini): port this
  // self_test_.add( "Connect Test", this, &Camera3D::Camera3D::ConnectTest );
  // diagnostic_.add( freq_diag_ );
  // diagnostic_.add( "Device Status", this, &Camera3D::Camera3D::deviceStatus );
  // TODO(giafranchini): port this

  max_delay = 1.0 / FOTONIC_DESIRED_FREQ;

	t = std::thread([this]{this->loop();});

	RCLCPP_INFO(get_logger(), "Fotonic 3D Camera component constructed");
}

Camera3D::~Camera3D()
{
  stop();
	t.join();
}


void Camera3D::InitSensor()
{
  RCLCPP_INFO(get_logger(), "Initializing sensor");
  double dMSShutter2 = 5.0;       // must be <= dShutter
  int iMSSaturation = 800;

  camera_->SetShutter( (unsigned short)( sensor_shutter_ms_ * 10 ) );
  camera_->SetFrameRate(
    (unsigned short) sensor_framerate_,
    (unsigned short) sensor_framerate_divisor_);
  camera_->SetMSShutter2( (unsigned short)( dMSShutter2 * 10 ) );
  camera_->SetMSSaturation( (unsigned short) iMSSaturation);

  camera_->SetFiltering(
    (unsigned short) lerp_filter_enable_, (unsigned short) edge_filter_enable_,
    edge_filter_minB_, edge_filter_diff1_, edge_filter_diff2_, edge_filter_diff3_);

  RCLCPP_INFO(get_logger(), "Sensor Shutter: %f", sensor_shutter_ms_);
  RCLCPP_INFO(get_logger(), "Sensor Frame Rate: %d", sensor_framerate_);
  RCLCPP_INFO(get_logger(), "Sensor Frame Rate Divisor: %d", sensor_framerate_divisor_);
  RCLCPP_INFO(get_logger(), "Lerp Filter: %d", lerp_filter_enable_);
  RCLCPP_INFO(get_logger(), "Edge Filter: %d", edge_filter_enable_);
}


int Camera3D::start()
{
  // Perform a configurable stop in order to delay the start up and don't saturate the Ethernet comunication
  Camera3D::stop();
  if (startup_delay_) {
    RCLCPP_WARN(get_logger(), "Delaying startup %d seconds...", startup_delay_);
    rclcpp::sleep_for(std::chrono::seconds(startup_delay_));
  }

  // Create camera object
  camera_ = new C_FZCamera();

  // Initialize library
  FZ_Result res = camera_->Init();

  RCLCPP_DEBUG(get_logger(), "FZ_Init returned %d", (int) res);

  // Initializes the camera with the given address
  char buf[50] = "/0";
  sprintf(buf, "%s", ip_address_.c_str() );

  // TODO(giafranchini): we need to do some error handling here, we cannot return -1 because we will not use
  // this inside spin()
  if (!camera_->SetActive(buf) ) {
    RCLCPP_ERROR(get_logger(), "SetActive failed");
    return -1;
  } else {
    RCLCPP_INFO(get_logger(), "Set active camera at %s", ip_address_.c_str());
  }

  // Sets camera measurement mode
  if (!camera_->SetMode(DE_MODE_PA_Z) ) {
    RCLCPP_ERROR(get_logger(), "SetMode failed");
    return -1;
  }

  InitSensor();
  camera_->Start();
  if (camera_->IsStarted()) {
    RCLCPP_INFO(get_logger(), "Camera Started");
  }


  // Enumerate Camera3D::Camera3D devices (This service is not working in the library, it also does not work in Windows)
  /*if (camera_->Enumerate() == 0){
                                RCLCPP_ERROR(get_logger(),"Camera3D::Camera3D::start - No device found!");
                                return -1;
                }*/

  RCLCPP_INFO(get_logger(), "Connected to 3D CAMERA");
  // freq_diag_.clear();

  // Get and print some important data to the connected camera
  char cameraData[100];

  camera_->GetInfoString(API_VERSION, cameraData, 50);
  camera_->GetInfoString(DE_VERSION, cameraData, 50);
  camera_->GetInfoString(CA_VERSION, cameraData, 50);
  RCLCPP_INFO(get_logger(), "API VERSION: %s", cameraData);
  RCLCPP_INFO(get_logger(), "DE VERSION: %s", cameraData);
  RCLCPP_INFO(get_logger(), "CA VERSION: %s", cameraData);
  running = true;
  return 0;
}

int Camera3D::stop()
{
  if (running) {
    // Closing actions
    delete camera_;
    running = false;
  }
  return 0;
}

// determines if a given pixel shall be filtered out or not
// X: if Z value of 1 horizontal neighbour differ more than "depth" the pixel is filtered
// Y: if Z value of 1 vertical neighbour differ more than "depth" the pixel is filtered
bool Camera3D::PreZFilterPixel(short * pPixels, int iNumX, int iNumY, int x, int y)
{
  // filtering enabled?
  if (!pre_zfilterx_enable_ && !pre_zfiltery_enable_) {
    return false;
  }

  // remove pixels where the filter radius is outside of image
  if (x <= 0 || x >= (iNumX - 1) || y <= 0 || y >= (iNumY - 1)) {
    return true;
  }

  // current position and pixel
  int iRowPosZ = (y * iNumX) * 4 + (iNumX * 1);
  short iZ = *(pPixels + iRowPosZ + x);

  if (pre_zfilterx_enable_) {
    // get surounding pixels
    short iZLeft = *(pPixels + iRowPosZ + x - 1);
    short iZRight = *(pPixels + iRowPosZ + x + 1);
    // check pixels to left and right
    if (iZ < (iZLeft - pre_zfilter_depth_) || iZ > (iZLeft + pre_zfilter_depth_) ||
      iZ < (iZRight - pre_zfilter_depth_) || iZ > (iZRight + pre_zfilter_depth_))
    {
      return true;
    }
  }

  if (pre_zfiltery_enable_) {
    int iRowPosZAbove = ((y - 1) * iNumX) * 4 + (iNumX * 1);
    int iRowPosZBelow = ((y + 1) * iNumX) * 4 + (iNumX * 1);
    // get surounding pixels
    short iZAbove = *(pPixels + iRowPosZAbove + x);
    short iZBelow = *(pPixels + iRowPosZBelow + x);
    // check pixels above and below
    if (iZ < (iZAbove - pre_zfilter_depth_) || iZ > (iZAbove + pre_zfilter_depth_) ||
      iZ < (iZBelow - pre_zfilter_depth_) || iZ > (iZBelow + pre_zfilter_depth_))
    {
      return true;
    }
  }

  return false;
}

// determines if a given pixel shall be filtered out or not
// if Z difference of neighbouring pixels within "radius" is more than "depth" a counter is increased
// if the counter reaches "pixels" the pixel is filtered
bool Camera3D::NoiseFilterPixel(short * pPixels, int iNumX, int iNumY, int x, int y)
{
  if (!noise_filter_enable_) {
    return false;
  }

  // remove pixels where the filter radius is outside of image
  if (x - noise_filter_radius_ < 0 || x + noise_filter_radius_ >= (iNumX - 1) ||
    y - noise_filter_radius_ < 0 || y + noise_filter_radius_ >= (iNumY - 1))
  {
    return true;
  }

  // current position and pixel
  int iRowPosZ = (y * iNumX) * 4 + (iNumX * 1);
  short iZ = *(pPixels + iRowPosZ + x);

  int iPixelsFound = 0;
  int iNumPixels = noise_filter_radius_ * 2 + 1;
  iNumPixels = iNumPixels * iNumPixels - 1;

  // each row in filter
  for (int iFilterY = -noise_filter_radius_; iFilterY <= noise_filter_radius_; iFilterY++) {
    // pointer to current row in filter
    int iFilterRowPosZ = ((y + iFilterY) * iNumX) * 4 + (iNumX * 1);

    for (int iFilterX = -noise_filter_radius_; iFilterX <= noise_filter_radius_; iFilterX++) {
      // dont use current pixel
      if (iFilterY == 0 && iFilterX == 0) {continue;}
      // get the neighbour pixel
      int iNeighbourPixel = *(pPixels + iFilterRowPosZ + x + iFilterX);
      // compare if current pixel is within depth of neighbour
      if ( (iNeighbourPixel - noise_filter_depth_) <= iZ &&
        iZ <= (iNeighbourPixel + noise_filter_depth_) )
      {
        iPixelsFound++;
        if (iPixelsFound >= noise_filter_pixels_) {
          // pixel will not be filtered
          return false;
        }
      }
      iNumPixels--;                   //pixels left to test
      // no way for the pixels needed to be found, exit early (pixel will be filtered)
      if (iNumPixels + iPixelsFound < noise_filter_pixels_) {
        return true;
      }
    }
  }
  // dont draw
  return true;       //this shall never happen
}


void Camera3D::getData()
{

  // Get pointclouds and intensities as image
  sensor_msgs::msg::PointCloud2 point_cloud;
  short * pImage = NULL;
  short * pImageLastOK = NULL;

  int iRows = 120;        // finally working !
  int iCols = 160;        //

  if (!camera_->IsStarted()) {
    return;
  }

  // Get Image and Image Header
  pImage = camera_->GetFrame();

  if (pImage) {
    FZ_FRAME_HEADER * pHeader = camera_->GetFrameHeader();
    RCLCPP_DEBUG(
      get_logger(), "version=%d bytesperpixel=%d nrows=%d ncols=%d framecounter=%d",
      pHeader->version, pHeader->bytesperpixel, pHeader->nrows, pHeader->ncols,
      pHeader->framecounter);
    RCLCPP_DEBUG(
      get_logger(), "lasterrorframe=%d shutter=%d mode=%d reportedframerate=%d",
      pHeader->lasterrorframe, pHeader->shutter, pHeader->mode, pHeader->reportedframerate);
    RCLCPP_DEBUG(
      get_logger(), "measuredframerate=%d, camera chip temperature=%3.2f[C]",
      pHeader->measuredframerate, (float)pHeader->temperature * 0.1);
    // timestamp[0] Contains start of exposure in POSIX time (seconds)
    // timestamp[1] Contains the milliseconds part of start of exposure.
    // timestamp[2] Contains the duration of the complete exposure in milliseconds.
    // timestamp[3] Not used, will contain zero.

    // copy to local
    int iSize = pHeader->bytesperpixel * pHeader->ncols * pHeader->nrows;
    pImageLastOK = new short[iSize / 2];
    memcpy(pImageLastOK, pImage, iSize);

    // PointCloud type
    // Prepare the PointCloud message
    auto current_time = now();


    // PointCloud2 type
    // Prepare the message to publish
    point_cloud.header.stamp = current_time;
    point_cloud.header.frame_id = fotonic_frame_id_;
    point_cloud.height = iRows;
    point_cloud.width = iCols;

    // size of field is 3 for x, y, z and 3 for rgb and u,v
    point_cloud.fields.resize(3 + 3);
    point_cloud.fields[0].name = "x";
    point_cloud.fields[1].name = "y";
    point_cloud.fields[2].name = "z";
    point_cloud.fields[3].name = "rgb";
    point_cloud.fields[4].name = "u";
    point_cloud.fields[5].name = "v";
    int offset = 0;
    for (size_t d = 0; d < point_cloud.fields.size(); ++d, offset += 4) {
      point_cloud.fields[d].offset = offset;
      point_cloud.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
      point_cloud.fields[d].count = 1;
    }

    point_cloud.point_step = offset;
    point_cloud.row_step = point_cloud.point_step * iCols;
    point_cloud.is_bigendian = false;
    point_cloud.data.resize(iCols * iRows * point_cloud.point_step);               //act.point data, size is (row_step*height)

    // Image type
    cv::Mat * cv_float_image = new cv::Mat(iRows, iCols, CV_16UC1);
    cv::Mat * cv_float_image_depth = new cv::Mat(iRows, iCols, CV_16UC1);           // CV_32FC1);

    int iNumY = iRows;
    int iNumX = iCols;

    for (int y = 0; y < iNumY; y++) {
      int iRowPos = y * (iNumX * 4);
      //image
      // first lib versions E70P
      for (int x = 0; x < iNumX; x++) {

        // new E70P version 3D image is inverted left-right
        int16_t iB = pImageLastOK[iRowPos + iNumX * 0 + x];                     // B
        int16_t iZ = pImageLastOK[iRowPos + iNumX * 1 + x];                     // Z
        int16_t iX = pImageLastOK[iRowPos + iNumX * 2 + x];                     // X
        int16_t iY = pImageLastOK[iRowPos + iNumX * 3 + x];                     // Y

        // pre Z filter
        if (PreZFilterPixel(pImageLastOK, iNumX, iNumY, x, y)) {
          continue;
        }

        // noise filter
        if (NoiseFilterPixel(pImageLastOK, iNumX, iNumY, x, y) ) {
          continue;
        }

        // Confidence level
        if ((iB < active_brightness_min_) || (iB > active_brightness_max_) || (iZ < 50)) {
          continue;
        }

        // Z Range
        if ((iZ < z_range_min_) || (iZ > z_range_max_)) {
          continue;
        }

        // scale from [mm] to [m]
        float fZ = (float)(iZ) * 0.001f;
        float fX = (float)(iX) * 0.001f;
        float fY = (float)(iY) * 0.001f;
        float fB = (float)(iB);

        // locate point in meters
        geometry_msgs::msg::Point32 pt;
        pt.z = fZ;
        // first lib
        pt.y = fX;
        pt.x = fY;
        //pt.x = fX;
        //pt.y = fY;

        // Fill PointCloud2 data
        // first version of E70P lib
        int adr = y * iNumX + x;

        // new E70P lib 3d image is inverted left-right
        //memcpy(&point_cloud.data[adr*point_cloud.point_step + point_cloud.fields[0].offset], &fX, sizeof (float));
        //memcpy(&point_cloud.data[adr*point_cloud.point_step + point_cloud.fields[1].offset], &fY, sizeof (float));
        memcpy(
          &point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[0].offset], &fY,
          sizeof(float));
        memcpy(
          &point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[1].offset], &fX,
          sizeof(float));
        memcpy(
          &point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[2].offset], &fZ,
          sizeof(float));
        memcpy(
          &point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[3].offset], &fB,
          sizeof(float));

        // Fill image data
        //cv_float_image->at<float>(y, iCols - x) = fB;   // pImageLastOK[iRowPos+iNumX*0+x];  //B
        cv_float_image->at<int16_t>(y, iCols - x) = iB;

        cv_float_image_depth->at<uint16_t>(y, iCols - x) = -iZ;                         // in [mm]
      }
    }

    // Publish pointclouds
    fotonic_3Ddata_pub_->publish(point_cloud);

    // Publish image
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = current_time;
    out_msg.header.frame_id = fotonic_frame_id_;
    out_msg.encoding = sensor_msgs::image_encodings::MONO16;             // Or whatever
    out_msg.image = *cv_float_image;             // the cv::Mat
    // image_pub_->publish(out_msg.toImageMsg());
    pub_.publish(out_msg.toImageMsg());

    // Publish image depth
    out_msg.image = *cv_float_image_depth;             // the cv::Mat
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    //image_depth_pub_->publish(out_msg.toImageMsg());
    pub_depth_.publish(out_msg.toImageMsg());

    //Publish Camera Info
    camera_info_msg_.header = out_msg.header;
    camera_info_pub_->publish(camera_info_msg_);
    camera_info_depth_pub_->publish(camera_info_msg_);

    // Publish the camera temperature (could be moved to diagnostics)
    std_msgs::msg::Float32 temperature_msg;
    temperature_msg.data = (float)pHeader->temperature * 0.1;
    temperature_pub_->publish(temperature_msg);
    delete cv_float_image;
    delete cv_float_image_depth;
    delete pImageLastOK;
  }
}

int Camera3D::read_and_publish()
{
  static double prevtime = 0;
  double starttime = now().seconds();
  if (prevtime && prevtime - starttime > 0.05) {
    RCLCPP_WARN(
      get_logger(), "Full fotonic_3dcamera loop took %f ms. Nominal is 10ms.",
      1000 * (prevtime - starttime));
    was_slow_ = "Full fotonic_3dcamera loop was slow.";
    slow_count_++;
  }

  getData();

  double endtime = now().seconds();
  if (endtime - starttime > max_delay) {
    RCLCPP_WARN(
      get_logger(), "Gathering data took %f ms. Nominal is 10ms.",
      1000 * (endtime - starttime));
    was_slow_ = "Full fotonic_3dcamera loop was slow.";
    slow_count_++;
  }

  prevtime = starttime;
  starttime = now().seconds();
  endtime = now().seconds();

  if (endtime - starttime > max_delay) {
    RCLCPP_WARN(
      get_logger(), "Publishing took %f ms. Nominal is 10 ms.",
      1000 * (endtime - starttime));
    was_slow_ = "Full fotonic_3dcamera loop was slow.";
    slow_count_++;
  }

  // freq_diag_.tick();
  return 0;
}

// TODO(giafranchini): port this in a ROS2 manner
bool Camera3D::loop() {
	while(rclcpp::ok()) {
		if (start() == 0) {
			if (read_and_publish() < 0) {
				break;
			}
		}
	}

	RCLCPP_INFO(get_logger(), "Camera3D::Camera3D::spin - calling stop !");
	stop();
	return true;
}

// void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
// {
//      // connection test
//      // TBC
//      status.summary(0, "Connected successfully.");
// }

// void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
// {
//      if (!running)
//              status.summary(2, "fotonic_3dcamera is stopped");
//      else if (!was_slow_.empty())
//      {
//              status.summary(1, "Excessive delay");
//              was_slow_.clear();
//      }
//      else
//              status.summary(0, "fotonic_3dcamera is running");

//      status.add("Error count", error_count_);
//      status.add("Excessive delay", slow_count_);
// }

} // namespace fotonic
