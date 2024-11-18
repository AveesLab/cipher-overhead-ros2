// Copyright 2021 Evan Flynn
// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Evan Flynn nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef USB_CAM__USB_CAM_NODE_HPP_
#define USB_CAM__USB_CAM_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <std_msgs/msg/string.hpp>

#include "usb_cam/usb_cam.hpp"

#include <cryptopp/aes.h>
#include <cryptopp/filters.h>
#include <cryptopp/modes.h>
#include <cryptopp/osrng.h>
#include <cryptopp/hex.h>

#define _GUN_SOURCE
//#include "ta_aes.h"

extern "C" {
    #include <err.h>
    #include <stdio.h>
    #include <string.h>
    #include <tee_client_api.h>
//    #include <secure_storage_ta.h>
}


std::ostream & operator<<(std::ostream & ostr, const rclcpp::Time & tm)
{
  ostr << tm.nanoseconds();
  return ostr;
}


namespace usb_cam
{

class UsbCamNode : public rclcpp::Node
{
public:
  explicit UsbCamNode(const rclcpp::NodeOptions & node_options);
  ~UsbCamNode();

  void init();
  void get_params();
  void assign_params(const std::vector<rclcpp::Parameter> & parameters);
  void set_v4l2_params();
  void update();
  bool take_and_send_image();
  bool take_and_send_image_mjpeg();

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  UsbCam * m_camera;

  sensor_msgs::msg::Image::SharedPtr m_image_msg;
  sensor_msgs::msg::CompressedImage::UniquePtr m_compressed_img_msg;
  std::shared_ptr<image_transport::CameraPublisher> m_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_compressed_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_compressed_cam_info_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  parameters_t m_parameters;

  sensor_msgs::msg::CameraInfo::SharedPtr m_camera_info_msg;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info;

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_service_capture;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameters_callback_handle;

  bool tcp;
//  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
//  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr Enc_image_publisher;
  std::shared_ptr<image_transport::CameraPublisher> Enc_image_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  int keylength;

  struct test_ctx {
      TEEC_Context ctx;
      TEEC_Session sess;
  };

  CryptoPP::SecByteBlock key;
  CryptoPP::SecByteBlock iv;
  UsbCamNode::test_ctx ctx;
  void initialize_tee(UsbCamNode::test_ctx *ctx);

  TEEC_Result save_key(UsbCamNode::test_ctx *ctx, char *id, char *data, size_t data_len);
  TEEC_Result load_key(UsbCamNode::test_ctx *ctx, char *id, char *data, size_t data_len);
  std::string encrypt(const std::string& plaintext);
  std::string encrypt_image_data(const std::string &serialized_image, size_t key_len);
  void EncryptionCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  char id[7] = "key_id";


};
}  // namespace usb_cam
#endif  // USB_CAM__USB_CAM_NODE_HPP_
