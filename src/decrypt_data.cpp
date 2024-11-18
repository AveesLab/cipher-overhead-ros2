#include "decrypt_data.hpp"
//
namespace Decrypt {

Decryption::Decryption(Ui::MainWindow* ui)
       : Node("decryption_node", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)), ui_(ui) 
{
  this->get_parameter_or("qos_tcp", tcp, true);
  this->get_parameter_or("key_size", keylength, 32);
  key = CryptoPP::SecByteBlock(0x00, keylength);
  iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);

  std::fill(key.begin(), key.end(), 'A');
  std::fill(iv.begin(), iv.end(), 'A');
  rclcpp::QoS default_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  default_qos.best_effort();
//  default_qos.reliable();

  /************************/
  /* Ros Topic Subscriber */
  /************************/
//  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("encrypted_image", default_qos, std::bind(&Decryption::DecryptionCallback, this, std::placeholders::_1));
  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", default_qos, std::bind(&Decryption::DecryptionCallback, this, std::placeholders::_1));


  /***********************/
  /* Ros Topic Publisher */
  /***********************/
//  DecryptedImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("decrypted_image", default_qos);


  fps_file_.open("/home/avees/ros2_ws/file/fps_data.csv");
  if(!fps_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the file!");
  }
  else {
    fps_file_ << "FPS" << std::endl;
  }

  e2e_file_.open("/home/avees/ros2_ws/file/e2e_delay.csv");
  if(!e2e_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the e2e file!");
  }
  else {
    e2e_file_ << "delay" << std::endl;
  }

}

std::string Decryption::decrypt(const std::string& ciphertext)
{
    std::string decryptedtext;
//
    CryptoPP::AES::Decryption aesDecryption(key, keylength);
    CryptoPP::CBC_Mode_ExternalCipher::Decryption cbcDecryption(aesDecryption, iv);
    CryptoPP::StreamTransformationFilter stfDecryptor(cbcDecryption, new CryptoPP::StringSink(decryptedtext));
    stfDecryptor.Put(reinterpret_cast<const unsigned char*>(ciphertext.c_str()), ciphertext.size());
    stfDecryptor.MessageEnd();
    
//    if(decryptedtext != ""){
//      RCLCPP_INFO(this->get_logger(), "Keysize = %i", keylength);
//    }
    
    return decryptedtext;
    
}

void Decryption::DecryptionCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if(!fps_file_.is_open()){
      std::cerr << "Failed to open the file!" << std::endl;
      return;
    }

//    // 프레임 타이밍 계산
    auto current_time = this->get_clock()->now();
    if (last_frame_time_.nanoseconds() != 0) {
        auto frame_duration = current_time - last_frame_time_;
        double fps = 1.0 / frame_duration.seconds();
//        std::cout << "FPS: " << fps << std::endl;

	ui_->fpsLabel->setText(QString("%1").arg(fps, 0, 'f', 2));
    	
	fps_file_ << fps << std::endl;
	if (++fps_counter_ >= 3000) {
	  fps_file_.close();
	  std::cout << "3000 FPS values recorded. Exiting." << std::endl;
//	  rclcpp::shutdown();
//	  return;
	}
    }

    last_frame_time_ = current_time;


//    // 암호화된 데이터를 문자열로 변환
//    std::string encrypted_msg(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
//
//    // 복호화
//    std::string decrypted_msg = decrypt(encrypted_msg);
//
//    // 암호화된 메시지 게시
//    auto decrypted_image_msg = std::make_shared<sensor_msgs::msg::Image>(*msg);
//    decrypted_image_msg->data.assign(decrypted_msg.begin(), decrypted_msg.end());
//
////    DecryptedImagePublisher_->publish(*decrypted_image_msg);
//
//    cv_bridge::CvImagePtr cv_ptr;
//    try{
//      cv_ptr = cv_bridge::toCvCopy(decrypted_image_msg, "bgr8");
//    }
//    catch(cv_bridge::Exception& e){
//      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//      return;
//    }

    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch(cv_bridge::Exception& e){
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat& image = cv_ptr->image;
    QImage qimage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
    QImage rgb_image = qimage.rgbSwapped();

    ui_->cameraLabel->setPixmap(QPixmap::fromImage(rgb_image));

    if(!e2e_file_.is_open()){
      std::cerr << "Failed to open the e2e file!" << std::endl;
      return;
    }
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    auto receive_time = system_clock.now();
    rclcpp::Time publish_time(msg->header.stamp, RCL_SYSTEM_TIME);

    rclcpp::Duration delay_duration = receive_time - publish_time;
    auto delay_ms = delay_duration.nanoseconds() / 1e6;
//    RCLCPP_INFO(this->get_logger(), "E2E delay: %.3f ms", delay_ms);
//    std::cout << "E2E delay: " << delay_ms << std::endl;

    if (delay_ms != 0){
      e2e_file_ << delay_ms << std::endl;
      if (++delay_counter_ >= 3000) {
        e2e_file_.close();
	std::cout << "3000 delay values recorded. Exiting." << std::endl;
	rclcpp::shutdown();
	return;
      }
    }

}

} /* namespace */

int main(int argc, char* argv[]){
    QApplication app(argc, argv);

    QMainWindow main_window;
    Ui::MainWindow ui;
    ui.setupUi(&main_window);

    QLabel *backgroundLabel = new QLabel(&main_window);
    QPixmap background(":/ui/aa.png");

    backgroundLabel->setPixmap(background.scaled(main_window.size(), Qt::IgnoreAspectRatio));
    backgroundLabel->setGeometry(0, 0, main_window.width(), main_window.height());
    backgroundLabel->lower();

    main_window.show();
//    label.setWindowTitle("QT Camera image");
//    label.resize(640, 480);
//    label.show();

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<Decrypt::Decryption>(&ui);

    std::thread ros_thread([&](){
      rclcpp::spin(node);
    });

    int result = app.exec();

//    cv::destroyAllWindows();
//    rclcpp::spin(node);
    rclcpp::shutdown();
    ros_thread.join();

    return result;
//    return 0;
}

