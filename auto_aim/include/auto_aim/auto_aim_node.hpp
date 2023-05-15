#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <ma_vision_interfaces/msg/armor.hpp>
#include <ma_vision_interfaces/msg/armors.hpp>

#include "detector_openvino/detect.hpp"


namespace MA {
class AutoAimNode : public rclcpp::Node {

public:
  explicit AutoAimNode(const rclcpp::NodeOptions &option);

protected:
  void publishMarkers();

protected:
  // sub camera-info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  // camera-info
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  image_transport::Publisher draw_img_pub_;


  rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;

  std::shared_ptr<image_transport::Subscriber> img_sub_;
  // 订阅相机图像的回调函数
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  cv::Point2f cam_center_;

  // Detected armors publisher
  ma_vision_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<ma_vision_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;


  std::unique_ptr<Detector> detector_;
};
} // namespace MA
