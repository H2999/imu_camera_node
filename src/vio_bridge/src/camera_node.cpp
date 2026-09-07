#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <deque>

class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    // 修改为相机实际支持的参数
    this->declare_parameter<int>("device_id", 2);
    this->declare_parameter<int>("width", 1280);
    this->declare_parameter<int>("height", 720);
    this->declare_parameter<int>("fps", 30);
    this->declare_parameter<std::string>("camera_frame_id", "camera_optical_frame");
    this->declare_parameter<std::string>("strb_topic", "/cam0/strb_stamp");
    this->declare_parameter<bool>("hardware_sync", true);

    int device_id = this->get_parameter("device_id").as_int();
    int width = this->get_parameter("width").as_int();
    int height = this->get_parameter("height").as_int();
    int fps = this->get_parameter("fps").as_int();
    frame_id_ = this->get_parameter("camera_frame_id").as_string();
    strb_topic_ = this->get_parameter("strb_topic").as_string();
    hardware_sync_ = this->get_parameter("hardware_sync").as_bool();

    // 打开相机（使用 V4L2 后端）
    cap_.open(device_id, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera device %d", device_id);
      return;
    }

    // 设置相机参数
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // 或 'Y', 'U', 'Y', 'V' 根据相机支持情况
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, fps);

    RCLCPP_INFO(this->get_logger(), "Camera initialized: %dx%dx@%dFPS", width, height, fps);

    // 初始化 Image Publisher
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", 10);
    if (hardware_sync_) {
      strb_sub_ = this->create_subscription<std_msgs::msg::Header>(
        strb_topic_, rclcpp::QoS(rclcpp::KeepLast(30)),
        std::bind(&CameraNode::strbCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Hardware sync enabled, waiting for STRB");
    } else {
      RCLCPP_WARN(this->get_logger(), "Hardware sync disabled, using host timestamps");
    }

    // 创建定时器以指定的帧率循环抓取和发布图像
    // 使用 std::chrono 计算周期
    auto period = std::chrono::microseconds(1000000 / fps);
    timer_ = this->create_wall_timer(period, std::bind(&CameraNode::captureCallback, this));
  }

  ~CameraNode()
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void strbCallback(const std_msgs::msg::Header::SharedPtr msg)
  {
    if (pending_frames_.empty()) {
      pending_timestamps_.push_back(msg->stamp);
      return;
    }

    cv::Mat frame = pending_frames_.front();
    pending_frames_.pop_front();
    publishFrame(frame, msg->stamp);
  }

  void captureCallback()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(this->get_logger(), "Failed to grab frame from camera");
      return;
    }

    if (frame.empty()) {
      return;
    }

    if (!hardware_sync_) {
      publishFrame(frame, this->now());
      return;
    }

    pending_frames_.push_back(frame.clone());
    if (!pending_timestamps_.empty()) {
      auto stamp = pending_timestamps_.front();
      pending_timestamps_.pop_front();
      cv::Mat pending = pending_frames_.front();
      pending_frames_.pop_front();
      publishFrame(pending, stamp);
    }

    while (pending_frames_.size() > 5) {
      pending_frames_.pop_front();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping camera frame because no matching STRB timestamp arrived");
    }
  }

  void publishFrame(const cv::Mat &frame,
                    const builtin_interfaces::msg::Time &stamp)
  {
    // OV9281 是黑白相机，如果输出是彩色格式，需转为单通道灰度图
    cv::Mat gray_frame;
    if (frame.channels() == 3) {
      cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    } else {
      gray_frame = frame;
    }

    // 转换为 ROS2 图像消息
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;

    sensor_msgs::msg::Image::SharedPtr img_msg = 
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, gray_frame).toImageMsg();

    // 发布图像
    image_pub_->publish(*img_msg);
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr strb_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  std::string strb_topic_;
  bool hardware_sync_;
  std::deque<cv::Mat> pending_frames_;
  std::deque<builtin_interfaces::msg::Time> pending_timestamps_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
