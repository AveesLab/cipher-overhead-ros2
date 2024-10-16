#pragma once

// C++
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <algorithm>
#include <limits>
#include <random>
#include <condition_variable>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/opencv.hpp>
#include <QApplication>
#include <QLabel>
#include <QPixmap>
#include <QBitmap>
#include <QImage>
#include <QWidget>
#include <QVBoxLayout>
#include <QPainter>
#include <QThread>
#include "ui_cluster.h"

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cryptopp/aes.h>
#include <cryptopp/filters.h>
#include <cryptopp/modes.h>
#include <cryptopp/osrng.h>
#include <cryptopp/hex.h>


#define _GUN_SOURCE

extern "C" {
    #include <err.h>
    #include <stdio.h>
    #include <string.h>
    #include <tee_client_api.h>
//  #include <secure_storage_ta.h>
}

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

namespace Decrypt {

class Decryption : public QObject, public rclcpp::Node
{

  Q_OBJECT

public:
  Decryption(Ui::MainWindow* ui);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ImageSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr DecryptedImagePublisher_;

  CryptoPP::SecByteBlock key;
  CryptoPP::SecByteBlock iv;
//  std::fill(key.begin(), key.end(), 'A');
//  std::fill(iv.begin(), iv.end(), 'A');

  bool tcp;
  int keylength;

  struct test_ctx {
      TEEC_Context ctx;
      TEEC_Session sess;
  };

  Decryption::test_ctx ctx;
  void initialize_tee(Decryption::test_ctx *ctx);
  void initialize_aes();
  TEEC_Result save_key(Decryption::test_ctx *ctx, char *id, char *data, size_t data_len);
  TEEC_Result load_key(Decryption::test_ctx *ctx, char *id, char *data, size_t data_len);
  std::string decrypt(const std::string& plaintext);
  void DecryptionCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  char id[7] = "key_id";
  
  Ui::MainWindow* ui_;
//  QLabel* cameraLabel_;

private:

};

}
