#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <thread>

#include "serial_test/serial.hpp"

#include "ma_vision_interfaces/msg/serial_send_data.hpp"

namespace MA {

class SerialTestNode : public rclcpp::Node {
public:
  explicit SerialTestNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("serial_test", options) {

    RCLCPP_INFO(this->get_logger(), "start Serial Test Node");

    // 串口配置
    serial_ = std::make_unique<Serial>();
    serial_->serial_name = "/dev/ttyACM0";
    serial_->password = "123";
    serial_->baudrate = 115200;

    //
    this->create_subscription<ma_vision_interfaces::msg::SerialSendData>(
        "/ma_vision/send_data", 2,
        std::bind(&SerialTestNode::sendDataCallback, this,
                  std::placeholders::_1));

    // tf
    tfGimbal2World = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tfCamera2Gimbal =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    // 相机 到 云台坐标系
    sendCamera2Gimbal();

    // 后台线程
    readThread = std::thread{[this]() {
      while (rclcpp::ok()) {
        /*
        if(!serial_->isOpen()){
          Respone res =  serial_->open();
          if(!res.success)
          {
            RCLCPP_WARN(this->get_logger(), "%s" ,res.reason.c_str());
            continue;
          }
        }
        Result<Serial::ReceiveData> res =  serial_->read();
        if(!res.success){
          RCLCPP_WARN(this->get_logger(), "%s" ,res.reason.c_str());
          continue;
        }
        Eigen::Isometry3d i_g2w = getRotationGimbal2World(res.res);
        */

        // 测试部分----------------------------------
        Eigen::Vector3d t(0, 0, 0);
        int times = 100;
        // [-PI, PI)
        double r_angle = i * 1.0f / times * M_PI;
        // r_angle = M_PI_4;
        if (addI) {
          if (i++ >= times - 5) {
            addI = false;
          }
        } else {
          if (i-- <= -times + 5) {
            addI = true;
          }
        }
        Eigen::Vector3d r_axis(0, 0, 1);
        Eigen::AngleAxisd r(r_angle, r_axis);

        i_g2w.translation() = t;
        i_g2w.linear() = r.toRotationMatrix().inverse();
        //--------------

        geometry_msgs::msg::TransformStamped transG2W =
            tf2::eigenToTransform(i_g2w);
        transG2W.header.stamp = now();
        transG2W.header.frame_id = "world";
        transG2W.child_frame_id = "gimbal";

        tfGimbal2World->sendTransform(transG2W);
        RCLCPP_INFO(this->get_logger(), "send transform");

        using namespace std::chrono_literals;
        // 测试需要休眠来实现定时，避免太快
        std::this_thread::sleep_for(10ms);
      }
    }};

    RCLCPP_INFO(this->get_logger(), "success!");
  }

  void pose_timer_test() {
    RCLCPP_INFO(this->get_logger(), "get IMU");

    Eigen::Vector3d t(0, 0, 0);
    int times = 100;
    // [-PI, PI)
    double r_angle = i * 1.0f / times * M_PI;
    // r_angle = M_PI_4;
    if (addI) {
      if (i++ >= times - 5) {
        addI = false;
      }
    } else {
      if (i-- <= -times + 5) {
        addI = true;
      }
    }
    Eigen::Vector3d r_axis(0, 0, 1);
    Eigen::AngleAxisd r(r_angle, r_axis);

    // Eigen::Isometry3d i_g2w;
    i_g2w.translation() = t;
    i_g2w.linear() = r.toRotationMatrix().inverse();
    geometry_msgs::msg::TransformStamped transG2W =
        tf2::eigenToTransform(i_g2w);
    transG2W.header.stamp = now();
    transG2W.header.frame_id = "world";
    transG2W.child_frame_id = "gimbal";

    tfGimbal2World->sendTransform(transG2W);
  }

  // TODO：补全
  Eigen::Isometry3d getRotationGimbal2World(Serial::ReceiveData data) {
    
  }

  void
  sendDataCallback(const ma_vision_interfaces::msg::SerialSendData &send_data) {

    RCLCPP_INFO(this->get_logger(),
                "get Serial Send Data: [yaw=%.3f, pitch=%.3f]",
                send_data.send_yaw, send_data.send_pitch);
    if (!serial_->isOpen()) {
      Respone res = serial_->open();
      if (!res.success) {
        RCLCPP_WARN(this->get_logger(), "%s", res.reason.c_str());
        return;
      }
    }

    Respone res = serial_->write(Serial::SendData{send_data.send_yaw, send_data.send_pitch});
    if(!res.success)
    {
      RCLCPP_WARN(this->get_logger(), "Serial Write data Error!");
    }
  }

  // 相机到云台坐标系的静态变换
  void sendCamera2Gimbal() {
    Eigen::Vector3d t(0, 0, 0);

    // 根据安装差异来
    Eigen::Matrix3d r;
    r << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    Eigen::Isometry3d i_g2w;
    i_g2w.translation() = t;
    i_g2w.linear() = r;
    geometry_msgs::msg::TransformStamped transG2W =
        tf2::eigenToTransform(i_g2w);
    transG2W.header.frame_id = "gimbal";
    transG2W.child_frame_id = "camera";

    tfCamera2Gimbal->sendTransform(transG2W);
  }

  ~SerialTestNode() override {
    if (readThread.joinable()) {
      readThread.join();
    }

    RCLCPP_INFO(this->get_logger(), "Serial Node destory!");
  }

public:
  // rclcpp::TimerBase::SharedPtr image_timer_;
  // rclcpp::TimerBase::SharedPtr pose_timer_;

  Eigen::Isometry3d i_g2w;

  std::thread readThread;

  std::unique_ptr<Serial> serial_;

  int i = 0;
  bool addI = true;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfGimbal2World;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfCamera2Gimbal;

  // 订阅话题，发送串口
  rclcpp::Subscription<ma_vision_interfaces::msg::SerialSendData>::SharedPtr
      send_serial_sub_;
};
} // namespace MA

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(MA::SerialTestNode)