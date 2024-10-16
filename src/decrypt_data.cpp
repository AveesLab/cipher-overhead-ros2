#include "decrypt_data.hpp"
//
//temp uuid
#define TA_SECURE_STORAGE_UUID \
	{0xf4e750bb, 0x1437, 0x4fbf, \
		{0x87, 0x85, 0x8d, 0x35, 0x80, 0xc3, 0x49, 0x93}}
namespace Decrypt {

Decryption::Decryption(Ui::MainWindow* ui)
       : Node("decryption_node", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)), ui_(ui) 
{
  this->get_parameter_or("key_size", keylength, 16);
  this->get_parameter_or("qos_tcp", tcp, false);
  key = CryptoPP::SecByteBlock(0x00, keylength);
  iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);
  initialize_tee(&ctx);

  // Initialize AES context and set up encryption parameters
  initialize_aes();

  save_key(&ctx, id, reinterpret_cast<char*>(key.data()), key.size());
  rclcpp::QoS default_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  default_qos.best_effort();
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  if(tcp){
    qos.reliable();
  }
  else qos.best_effort();

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("encrypted_image", qos, std::bind(&Decryption::DecryptionCallback, this, std::placeholders::_1));


  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  DecryptedImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("decrypted_image", default_qos);
}

void Decryption::initialize_tee(Decryption::test_ctx *ctx)
{
    TEEC_UUID uuid = TA_SECURE_STORAGE_UUID;
    uint32_t origin;
    TEEC_Result res;

    res = TEEC_InitializeContext(NULL, &ctx->ctx);
    if (res != TEEC_SUCCESS)
	errx(1, "TEEC_InitializeContext failed with code 0x%x", res);

    res = TEEC_OpenSession(&ctx->ctx, &ctx->sess, &uuid, TEEC_LOGIN_PUBLIC, NULL, NULL, &origin);
    if (res != TEEC_SUCCESS)
	    errx(1, "TEEC_Opensession failed with code 0x%x origin 0x%x", res, origin);
}
  
void Decryption::initialize_aes()
{
//    CryptoPP::SecByteBlock key = CryptoPP::SecByteBlock(0x00, keylength);
//    CryptoPP::SecByteBlock iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);

//    CryptoPP::AutoSeededRandomPool prng;
//    prng.GenerateBlock(key, key.size());
//    prng.GenerateBlock(iv, iv.size());
    std::fill(key.begin(), key.end(), 'A');
    std::fill(iv.begin(), iv.end(), 'A');

//    CryptoPP::SecByteBlock key = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::DEFAULT_KEYLENGTH);
//    CryptoPP::SecByteBlock iv = CryptoPP::SecByteBlock(0x00, CryptoPP::AES::BLOCKSIZE);
}

TEEC_Result Decryption::save_key(Decryption::test_ctx *ctx, char *id, char *data, size_t data_len)
{
    TEEC_Operation op;
    uint32_t origin;
    TEEC_Result res;
    size_t id_len = strlen(id);

    memset(&op, 0, sizeof(op));
    op.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
		    		     TEEC_MEMREF_TEMP_INPUT,
				     TEEC_NONE, TEEC_NONE);

    op.params[0].tmpref.buffer = id;
    op.params[0].tmpref.size = id_len;

    op.params[1].tmpref.buffer = data;
    op.params[1].tmpref.size = data_len;

    res = TEEC_InvokeCommand(&ctx->sess, 1, &op, &origin);
    memset(&op, 0, sizeof(op));

    if (res != TEEC_SUCCESS)
	errx(1, "Command WRITE failed: 0x%x / %u\n", res, origin);

    switch (res) {
    case TEEC_SUCCESS:
	break;
    default:
	errx(1, "Command WRITE failed: 0x%x / %u\n", res, origin);
    }
    return res;
}

TEEC_Result Decryption::load_key(Decryption::test_ctx *ctx, char *id, char *data, size_t data_len)
{
	TEEC_Operation op;
	uint32_t origin;
	TEEC_Result res;
	size_t id_len = strlen(id);

	memset(&op, 0, sizeof(op));
	op.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
					 TEEC_MEMREF_TEMP_OUTPUT,
					 TEEC_NONE, TEEC_NONE);

	op.params[0].tmpref.buffer = id;
	op.params[0].tmpref.size = id_len;

	op.params[1].tmpref.buffer = data;
	op.params[1].tmpref.size = data_len;

	res = TEEC_InvokeCommand(&ctx->sess, 0, &op, &origin);

	memset(&op, 0, sizeof(op));
	switch (res) {
	case TEEC_SUCCESS:
	case TEEC_ERROR_SHORT_BUFFER:
	case TEEC_ERROR_ITEM_NOT_FOUND:
	    break;
	default:
	    errx(1, "Command READ failed: 0x%x / %u\n", res, origin);
	}
	return res;
}

std::string Decryption::decrypt(const std::string& ciphertext)
{
    std::string decryptedtext;

    char saved_key[keylength];
    load_key(&ctx, id, saved_key, keylength);
    CryptoPP::SecByteBlock key_string(reinterpret_cast<const unsigned char*>(saved_key), strlen(saved_key));
    CryptoPP::AES::Decryption aesDecryption(key_string, keylength);
    //CryptoPP::AES::Decryption aesDecryption(key, keylength);
    CryptoPP::CBC_Mode_ExternalCipher::Decryption cbcDecryption(aesDecryption, iv);
    CryptoPP::StreamTransformationFilter stfDecryptor(cbcDecryption, new CryptoPP::StringSink(decryptedtext));
    stfDecryptor.Put(reinterpret_cast<const unsigned char*>(ciphertext.c_str()), ciphertext.size());
    stfDecryptor.MessageEnd();
    
    if(decryptedtext != ""){
      RCLCPP_INFO(this->get_logger(), "Keysize = %i", keylength);
    }
    
    return decryptedtext;
    
}

void Decryption::DecryptionCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

//    // 프레임 타이밍 계산
//    auto current_time = this->get_clock()->now();
//    if (last_frame_time_.nanoseconds() != 0) {
//        auto frame_duration = current_time - last_frame_time_;
//        double fps = 1.0 / frame_duration.seconds();
//        std::cout << "FPS: " << fps << std::endl;
//    }
//    last_frame_time_ = current_time;

    // 암호화된 데이터를 문자열로 변환
    std::string encrypted_msg(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());

    // 복호화
    std::string decrypted_msg = decrypt(encrypted_msg);

    // 암호화된 메시지 게시
    auto decrypted_image_msg = std::make_shared<sensor_msgs::msg::Image>(*msg);
    decrypted_image_msg->data.assign(decrypted_msg.begin(), decrypted_msg.end());

    DecryptedImagePublisher_->publish(*decrypted_image_msg);

    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(decrypted_image_msg, "bgr8");
    }
    catch(cv_bridge::Exception& e){
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat& image = cv_ptr->image;
//    cv::Mat mask = cv::Mat::zeros(image.size(), image.type());
//    int radius = std::min(image.cols, image.rows) / 2;
//    cv::Point center(image.cols / 2, image.rows / 2);
//    cv::circle(mask, center, radius, cv::Scalar(255,255,255), -1);
//
//    cv::Mat circular_image;
//    image.copyTo(circular_image, mask);

//    QImage qimage(circular_image.data, circular_image.cols, circular_image.rows, circular_image.step, QImage::Format_RGB888);
    QImage qimage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
    QImage rgb_image = qimage.rgbSwapped();

    ui_->cameraLabel->setPixmap(QPixmap::fromImage(rgb_image));
//    label_->setPixmap(QPixmap::fromImage(rgbImage));


//    cv::imshow("Camera Image", cv_ptr->image);
//    cv::waitKey(1);
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

