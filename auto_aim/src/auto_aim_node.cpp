#include "auto_aim/auto_aim_node.hpp"

#include <fmt/core.h>
#include <opencv2/opencv.hpp>

namespace MA {

AutoAimNode::AutoAimNode(const rclcpp::NodeOptions &option)
    : rclcpp::Node("auto_aim", option) {

  // 参数初始化
  std::string cam_info_topic_name_ = "/camera_info";
  std::string img_topic_name_ = "/mv_vision";

  // 订阅 camera info
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      cam_info_topic_name_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) -> void {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ =
            std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        cam_info_sub_.reset(); // 只订阅一次
      });

  // cam_info_->k, 9, 内参
  // cam_info_->d, 5, 畸变
  RCLCPP_INFO(this->get_logger(), "订阅 camera info: topic[%s]",
              cam_info_topic_name_.c_str());

  detector_ =
      std::make_unique<Detector>("/home/yiyu/CopyCode/rm_auto_aim/Configs/"
                                 "detect/armor-nano-poly-fp32-best.onnx");
  RCLCPP_INFO(this->get_logger(), "Init Detector");

  // 订阅 image
  img_sub_ = std::make_shared<image_transport::Subscriber>(
      image_transport::create_subscription(
          this, img_topic_name_,
          std::bind(&AutoAimNode::imageCallback, this, std::placeholders::_1),
          "raw"));

  RCLCPP_INFO(this->get_logger(), "订阅 image: topic[%s]",
              img_topic_name_.c_str());

  // debug
  draw_img_pub_ = image_transport::create_publisher(this, "/ma_vision/draw");
}

void AutoAimNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {

  RCLCPP_INFO(this->get_logger(), "[imageCallback] get Image");
  rclcpp::Time start_time = this->now();

  auto msg = cv_bridge::toCvShare(img_msg, "bgr8");
  cv::Mat image = msg->image;

  Detection_pack pack{image, this->now().seconds()};
  if (detector_)
    detector_->detect(pack);

  rclcpp::Time end_time = this->now();
  double latency = (end_time - start_time).seconds() * 1000;
  RCLCPP_INFO(this->get_logger(), "detectArmors used: %f ms", latency);

  cv::putText(image, "Latency: " + std::to_string(latency) + "ms",
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
              cv::Scalar(0, 255, 0), 2);

  cv::Mat drawImg = image.clone();
  cv::Scalar paint = cv::Scalar(255, 0, 0);
  for (const auto &armor : pack.armors) {
    auto fmt_str = fmt::format("Class: {}, Color: {}, Conf: {:.3f}",
                               ARMOR_CLASSES.at(armor.cls),
                               ARMOR_COLORS.at(armor.color), armor.prob);

    cv::putText(drawImg, fmt_str, armor.apex[0] - cv::Point2f(0, 10),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0));
    cv::line(drawImg, armor.apex[0], armor.apex[1], paint, 2);
    cv::line(drawImg, armor.apex[1], armor.apex[2], paint, 2);
    cv::line(drawImg, armor.apex[2], armor.apex[3], paint, 2);
    cv::line(drawImg, armor.apex[3], armor.apex[0], paint, 2);
  }

  draw_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "bgr8", drawImg).toImageMsg());

  RCLCPP_INFO(this->get_logger(), "publish debug image!");
}

} // namespace MA

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(MA::AutoAimNode)