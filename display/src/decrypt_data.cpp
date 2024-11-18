#include "decrypt_data.hpp"
#include "aes_ta.h"
//
namespace Decrypt {

Decryption::Decryption(Ui::MainWindow* ui)
       : Node("decryption_node", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)), ui_(ui) 
{
  this->get_parameter_or("qos_tcp", tcp, true);
  rclcpp::QoS default_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  default_qos.best_effort();
//  default_qos.reliable();
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  if(tcp){
    qos.reliable();
  }
  else qos.best_effort();

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("encrypted_image", default_qos, std::bind(&Decryption::DecryptionCallback, this, std::placeholders::_1));
//  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", default_qos, std::bind(&Decryption::DecryptionCallback, this, std::placeholders::_1));


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
	if (++fps_counter_ >= 5000) {
	  fps_file_.close();
	  std::cout << "5000 FPS values recorded. Exiting." << std::endl;
//	  rclcpp::shutdown();
//	  return;
	}
    }

    last_frame_time_ = current_time;
    size_t data_size = msg->data.size();

// 암호화된 메시지 데이터 복호화
    TEEC_UUID uuid = TA_AES_UUID;
    TEEC_Result res;
    uint32_t origin;
    TEEC_Operation op;

    // TEE 컨텍스트 및 세션 설정
    res = TEEC_InitializeContext(NULL, &ctx.ctx);
    if (res != TEEC_SUCCESS) {
        std::cerr << "TEE 컨텍스트 초기화 실패: 0x" << std::hex << res << std::endl;
        return;
    }

    res = TEEC_OpenSession(&ctx.ctx, &ctx.sess, &uuid, TEEC_LOGIN_PUBLIC, NULL, NULL, &origin);
    if (res != TEEC_SUCCESS) {
        std::cerr << "TEE 세션 열기 실패: 0x" << std::hex << res << std::endl;
        TEEC_FinalizeContext(&ctx.ctx);
        return;
    }

    // 복호화 준비
    memset(&op, 0, sizeof(op));
    op.paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INPUT, TEEC_VALUE_INPUT, TEEC_VALUE_INPUT, TEEC_NONE);
    op.params[0].value.a = TA_AES_ALGO_CTR;             // AES 모드
    op.params[1].value.a = AES_KEY_BYTE_SIZE;        // AES 키 크기
    op.params[2].value.a = TA_AES_MODE_DECODE;          // 복호화 모드

    res = TEEC_InvokeCommand(&ctx.sess, TA_AES_CMD_PREPARE, &op, &origin);
    if (res != TEEC_SUCCESS) {
        std::cerr << "복호화 준비 실패: 0x" << std::hex << res << std::endl;
        TEEC_CloseSession(&ctx.sess);
        TEEC_FinalizeContext(&ctx.ctx);
        return;
    }

    // IV 설정
    char iv[AES_BLOCK_SIZE] = {0};  // 암호화할 때 사용했던 동일한 IV 사용
    op.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT, TEEC_NONE, TEEC_NONE, TEEC_NONE);
    op.params[0].tmpref.buffer = iv;
    op.params[0].tmpref.size = AES_BLOCK_SIZE;

    res = TEEC_InvokeCommand(&ctx.sess, TA_AES_CMD_SET_IV, &op, &origin);
    if (res != TEEC_SUCCESS) {
        std::cerr << "IV 설정 실패: 0x" << std::hex << res << std::endl;
        TEEC_CloseSession(&ctx.sess);
        TEEC_FinalizeContext(&ctx.ctx);
        return;
    }

    // 암호화된 메시지 데이터를 복호화
    std::vector<char> decrypted_data(msg->data.size()); // 복호화된 데이터를 저장할 벡터
    memset(&op, 0, sizeof(op));
    op.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT, TEEC_MEMREF_TEMP_OUTPUT, TEEC_NONE, TEEC_NONE);
    op.params[0].tmpref.buffer = msg->data.data();
    op.params[0].tmpref.size = data_size;
    op.params[1].tmpref.buffer = decrypted_data.data();
    op.params[1].tmpref.size = decrypted_data.size();

    res = TEEC_InvokeCommand(&ctx.sess, TA_AES_CMD_CIPHER, &op, &origin);
    if (res != TEEC_SUCCESS) {
        std::cerr << "복호화 요청 실패: 0x" << std::hex << res << std::endl;
        TEEC_CloseSession(&ctx.sess);
        TEEC_FinalizeContext(&ctx.ctx);
        return;
    }

    size_t decrypted_size = op.params[1].tmpref.size;
//    std::cout << "복호화된 데이터 크기: " << decrypted_size << " bytes" << std::endl;

    // TEE 세션 및 컨텍스트 해제
    TEEC_CloseSession(&ctx.sess);
    TEEC_FinalizeContext(&ctx.ctx);

    // 복호화된 데이터를 이미지로 변환
    auto decrypted_image_msg = std::make_shared<sensor_msgs::msg::Image>(*msg);
    decrypted_image_msg->data.assign(decrypted_data.begin(), decrypted_data.end());

    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(decrypted_image_msg, "bgr8");
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

