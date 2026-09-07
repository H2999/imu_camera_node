#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <vector>
#include <sys/select.h>
#include <chrono>

using namespace std::chrono_literals;

#pragma pack(push, 1)
struct IMUPacket
{
    uint16_t header;
    uint64_t timestamp_us;
    uint64_t frame_id;
    float acc[3];
    float gyro[3];
    uint8_t trigger_flag;
    uint8_t strb_flag;
    uint64_t strb_timestamp_us;
    uint16_t crc;
};
#pragma pack(pop)

uint16_t crc16(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

class VIOBridgeNode : public rclcpp::Node
{
public:
    VIOBridgeNode() : Node("vio_bridge_node"), serial_fd_(-1)
    {
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 921600);
        this->declare_parameter<std::string>("imu_topic", "/imu0");
        this->declare_parameter<std::string>("strb_topic", "/cam0/strb_stamp");
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<int>("max_imu_gap_us", 50000);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        strb_topic_ = this->get_parameter("strb_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        max_imu_gap_us_ = this->get_parameter("max_imu_gap_us").as_int();

        // 打开串口
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        // 配置串口
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr 失败");
            return;
        }

        // 设置波特率
        speed_t speed;
        switch (baudrate) 
        {
            case 9600:   speed = B9600; break;
            case 19200:  speed = B19200; break;
            case 38400:  speed = B38400; break;
            case 57600:  speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default:     speed = B115200; break;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag &= ~PARENB;   // 无校验
        tty.c_cflag &= ~CSTOPB;   // 1位停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;        // 8位数据
        tty.c_cflag &= ~CRTSCTS;   // 无流控
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);

        tty.c_oflag &= ~OPOST;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed");
            rclcpp::shutdown();
            return;
        }
        // A restarted bridge must not feed buffered packets from a previous
        // session into VINS together with current camera timestamps.
        tcflush(serial_fd_, TCIFLUSH);

        RCLCPP_INFO(this->get_logger(), "串口打开成功: %s, 波特率: %d", port.c_str(), baudrate);

        // VINS-Fusion 默认配置订阅 /imu0 和 /cam0/image_raw。
        // 本节点发布 IMU 到 /imu0；相机图像仍需要由相机节点发布到 /cam0/image_raw。
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::QoS(rclcpp::KeepLast(100)));
        strb_pub_ = this->create_publisher<std_msgs::msg::Header>(
            strb_topic_, rclcpp::QoS(rclcpp::KeepLast(10)));

        // Read frequently enough that the serial driver cannot build a stale
        // backlog at the 200 Hz IMU rate.
        timer_ = this->create_wall_timer(1ms, [this](){this->read_callback();});

        RCLCPP_INFO(this->get_logger(),
                    "VIO Bridge 节点启动成功! IMU: %s, STRB: %s, frame_id: %s",
                    imu_topic_.c_str(), strb_topic_.c_str(), frame_id_.c_str());
    }

    ~VIOBridgeNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    void read_callback()
    {
        if (serial_fd_ < 0) return;

        // 使用静态缓冲区缓存未处理完的字节
        static std::vector<uint8_t> rx_buffer;
        uint8_t temp_buf[256];
        
        ssize_t n = read(serial_fd_, temp_buf, sizeof(temp_buf));
        if (n > 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "serial read %ld bytes, packet size %zu", n, sizeof(IMUPacket));
            rx_buffer.insert(rx_buffer.end(), temp_buf, temp_buf + n);
        }

        // 循环解析缓冲区中的数据包
        while (rx_buffer.size() >= sizeof(IMUPacket)) {
            // 1. 查找帧头 0xA5A5 (小端模式下，低字节在前，高字节在后，即 0xA5, 0xA5)
            // 假设 packet.header 是 uint16_t 0xA5A5
            size_t header_idx = 0;
            bool found = false;
            for (size_t i = 0; i <= rx_buffer.size() - sizeof(IMUPacket); ++i) {
                uint16_t hdr = *reinterpret_cast<const uint16_t*>(&rx_buffer[i]);
                if (hdr == 0xA5A5) {
                    header_idx = i;
                    found = true;
                    break;
                }
            }

            if (!found) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "no header found, rx_buffer size %zu", rx_buffer.size());
                // 如果没找到帧头，保留最后几个字节（防止帧头被截断），其余丢弃
                if (rx_buffer.size() > sizeof(IMUPacket)) {
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.end() - sizeof(IMUPacket));
                }
                break;
            }

            // 如果帧头不在开头，说明前面有垃圾数据，先清理掉
            if (header_idx > 0) {
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + header_idx);
            }

            // 此时缓冲区头部已经是对齐的帧头，检查是否有足够的数据包长度
            if (rx_buffer.size() < sizeof(IMUPacket)) {
                break; // 数据不够一包，等下次读取
            }

            // 提取一个完整包
            IMUPacket packet;
            std::memcpy(&packet, rx_buffer.data(), sizeof(IMUPacket));

            // 从缓冲区移除已处理的这一包
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + sizeof(IMUPacket));

            // 2. CRC 校验
            uint16_t calc_crc = crc16((uint8_t*)&packet, sizeof(IMUPacket) - 2);
            if (calc_crc != packet.crc) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "CRC校验失败，丢弃该包");
                continue;
            }

            // STRB is independent camera timing. Publish it before applying
            // IMU-order checks so a discarded IMU sample cannot block images.
            if (packet.strb_flag) {
                auto strb_msg = std_msgs::msg::Header();
                strb_msg.stamp.sec = packet.strb_timestamp_us / 1000000;
                strb_msg.stamp.nanosec =
                    (packet.strb_timestamp_us % 1000000) * 1000;
                strb_msg.frame_id = frame_id_;
                strb_pub_->publish(strb_msg);
            }

            if (last_imu_timestamp_us_ != 0 &&
                packet.timestamp_us <= last_imu_timestamp_us_) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Discarding non-increasing IMU timestamp: current=%lu last=%lu",
                    packet.timestamp_us, last_imu_timestamp_us_);
                continue;
            }
            const uint64_t imu_dt_us = last_imu_timestamp_us_ == 0
                ? 0
                : packet.timestamp_us - last_imu_timestamp_us_;
            if (last_imu_timestamp_us_ != 0 &&
                imu_dt_us > static_cast<uint64_t>(max_imu_gap_us_)) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Discarding IMU timestamp gap of %.3f ms; restart VINS before continuing",
                    static_cast<double>(imu_dt_us) / 1000.0);
                last_imu_timestamp_us_ = packet.timestamp_us;
                continue;
            }
            last_imu_timestamp_us_ = packet.timestamp_us;

            // 3. 发布 IMU 话题
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp.sec = packet.timestamp_us / 1000000;
            imu_msg.header.stamp.nanosec = (packet.timestamp_us % 1000000) * 1000;
            imu_msg.header.frame_id = frame_id_;

            imu_msg.linear_acceleration.x = packet.acc[0];
            imu_msg.linear_acceleration.y = packet.acc[1];
            imu_msg.linear_acceleration.z = packet.acc[2];

            imu_msg.angular_velocity.x = packet.gyro[0];
            imu_msg.angular_velocity.y = packet.gyro[1];
            imu_msg.angular_velocity.z = packet.gyro[2];

            // 协方差矩阵按需填写（VIO通常需要，如果单片机没有提供可以填0或设定固定值）
            imu_pub_->publish(imu_msg);
        }
    }

    int serial_fd_;
    std::string imu_topic_;
    std::string strb_topic_;
    std::string frame_id_;
    uint64_t last_imu_timestamp_us_ = 0;
    uint64_t max_imu_gap_us_ = 50000;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr strb_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VIOBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
