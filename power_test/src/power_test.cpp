#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

namespace MA {

struct BuffObject {
  cv::Point2f apexs[5];
  int cls;
  int color;
  float conf;
};

class PowerTestNode : public rclcpp::Node {
public:
  explicit PowerTestNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("power_test", options) {

    RCLCPP_INFO(this->get_logger(), "start Power Test Node");

    using namespace std::chrono_literals;
    image_timer_ = this->create_wall_timer(
        20ms, std::bind(&PowerTestNode::imageCallback, this));

    RCLCPP_INFO(this->get_logger(), "create wall timer");

    image_pub_ = std::make_unique<image_transport::Publisher>(
        image_transport::create_publisher(this, "/power_test"));

    RCLCPP_INFO(this->get_logger(), "create image publisher");

    // Visualization Marker Publisher
    position_marker.ns = "armors";
    position_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    position_marker.scale.x = position_marker.scale.y =
        position_marker.scale.z = 0.1;
    position_marker.color.a = 1.0;
    position_marker.color.r = 1.0;
    position_marker.header.frame_id = "world";

    text_marker.ns = "classification";
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.1;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.02);
    text_marker.header.frame_id = "world";

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/detector/marker", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // tf_buffer_->setUsingDedicatedThread(true);
    tfWorld2Power = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "success!");
  }

  void imageCallback() {
    geometry_msgs::msg::TransformStamped t;
    try {
      using namespace std::chrono_literals;
      t = tf_buffer_->lookupTransform("world", "camera", this->now(), 30ms);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s", "camera",
                  "world");
      return;
    }
    // 1. 拿到图片
    RCLCPP_INFO(this->get_logger(), "get image");
    cv::Mat image = cv::Mat::zeros(640, 640, CV_8UC3);

    // 2. 识别
    // std::vector<BuffObject> objects = detector.detect(image);

    // 3. 解算[拟合阶段、击打阶段]：
    // 找到待击打扇叶和中心R标, solvepnp求解出相机坐标系->云台坐标系->世界坐标系
    // 如已拟合曲线则直接使用，否则拟合
    Eigen::Vector3d r_world(3.2, 1, -0.3);
    Eigen::Vector3d n_world(3.2, 0, 0);

    Eigen::Vector3d test_camera(1, 1, 1);

    Eigen::Isometry3d c2w = tf2::transformToEigen(t);
    Eigen::Vector3d test_world =
        c2w.rotation() * test_camera + c2w.translation();

    // 4. 数据可视化(draw图像、markers)
    position_marker.points.clear();
    markers.markers.clear();
    // test
    {
      geometry_msgs::msg::Point position;
      position.x = test_world(0, 0);
      position.y = test_world(1, 0);
      position.z = test_world(2, 0);

      text_marker.pose.position = position;
      text_marker.pose.position.y -= 0.1;
      text_marker.text = "Test";
      text_marker.id++;

      position_marker.points.emplace_back(position);
      markers.markers.emplace_back(text_marker);
    }
    // R
    {
      geometry_msgs::msg::Point position;
      position.x = r_world(0, 0);
      position.y = r_world(1, 0);
      position.z = r_world(2, 0);

      text_marker.pose.position = position;
      text_marker.pose.position.y -= 0.1;
      text_marker.text = "R";
      text_marker.id++;

      position_marker.points.emplace_back(position);
      markers.markers.emplace_back(text_marker);
    }
    // 待激活
    {
      geometry_msgs::msg::Point position;
      position.x = n_world(0, 0);
      position.y = n_world(1, 0);
      position.z = n_world(2, 0);

      text_marker.pose.position = position;
      text_marker.pose.position.y -= 0.1;
      text_marker.text = "no";
      text_marker.id++;

      position_marker.points.emplace_back(position);
      markers.markers.emplace_back(text_marker);
    }

    markers.markers.emplace_back(position_marker);
    marker_pub_->publish(markers);

    // 这里可以使用img_msg 的 header
    std_msgs::msg::Header header;
    header.stamp = now();

    auto cv_image = cv_bridge::CvImage(header, "rgb8", image);
    image_pub_->publish(cv_image.toImageMsg());
  }

  rclcpp::TimerBase::SharedPtr image_timer_;

  std::unique_ptr<image_transport::Publisher> image_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfWorld2Power;

  visualization_msgs::msg::Marker position_marker;
  visualization_msgs::msg::Marker text_marker;
  visualization_msgs::msg::MarkerArray markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  Eigen::Isometry3d i_g2w;

  int i = 0;
  bool addI = true;
};
} // namespace MA

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(MA::PowerTestNode)