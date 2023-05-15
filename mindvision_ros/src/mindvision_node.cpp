// MVSDK
#include <CameraApi.h>

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// #include <imu_complementary_filter/complementary_filter_ros.h>

// C++
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace mindvision_camera {
#define MV_CHECK_API(expr, info, ...)                                          \
  do {                                                                         \
    auto status = (expr);                                                      \
    if (status != CAMERA_STATUS_SUCCESS) {                                     \
      RCLCPP_ERROR(this->get_logger(),                                         \
                   "'" #expr "', status = %d, '" info "', '"##__VA_ARGS__      \
                   "' ",                                                       \
                   status);                                                    \
      return;                                                                  \
    }                                                                          \
  } while (0)

class MVCameraNode : public rclcpp::Node {
public:
  explicit MVCameraNode(const rclcpp::NodeOptions &options)
      : Node("mv_camera", options) {
    // int status = open_camera();
    RCLCPP_INFO(this->get_logger(), "Starting  MVCameraNode!");

    // qos 策略
    bool is_buffer_model =
        this->declare_parameter<bool>("is_buffer_model", true);
    // 相机配置参数文件路径, 用于MV相机
    std::string camera_config_url = this->declare_parameter<std::string>(
        "camera_config_url", PROJECT_DIR "/config/MV-SUA133GC.config");
    // 相机参数文件路径, camera info,
    // 图像大小, 相机内参、畸变
    std::string camera_info_url = this->declare_parameter<std::string>(
        "camera_info_url", "package://mindvision_ros/config/camera_info.yaml");



    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    int i_camera_counts = 1;
    int i_status = -1;
    tSdkCameraDevInfo t_camera_enum_list;
    MV_CHECK_API(CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts),
                 "枚举设备");
    // RCLCPP_INFO(this->get_logger(), "Enumerate state = %d", i_status);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", i_camera_counts);

    // 无设备连接
    // if (i_camera_counts == 0) {
    //     RCLCPP_WARN(this->get_logger(), "No camera found!");
    //     return;
    // }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    MV_CHECK_API(CameraInit(&t_camera_enum_list, -1, -1, &h_camera_),
                 "初始化相机");
    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    MV_CHECK_API(CameraGetCapability(h_camera_, &t_capability_), "");

    // 直接使用vector的内存作为相机输出buffer
    image_msg_.data.reserve(t_capability_.sResolutionRange.iHeightMax *
                            t_capability_.sResolutionRange.iWidthMax * 3);

    // 设置手动曝光
    MV_CHECK_API(CameraSetAeState(h_camera_, false), "设置手动曝光");
    // 让SDK进入工作模式，开始接收来自相机发送的图像
    // 数据。如果当前相机是触发模式，则需要接收到
    // 触发帧以后才会更新图像。
    MV_CHECK_API(CameraPlay(h_camera_), "进入工作模式");
    // 设置 RGB
    MV_CHECK_API(CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_RGB8), "");

    // 选择qos策略
    // https://docs.ros.org/en/galactic/Concepts/About-Quality-of-Service-Settings.html
    auto qos =  rclcpp::SensorDataQoS(); //is_buffer_model ? qos_buffer : qos_least;

    camera_pub_ =
        image_transport::create_camera_publisher(this, "/mv_vision", rmw_qos_profile_default);

    // Load camera info
    camera_name_ =
        this->declare_parameter<std::string>("camera_name", "mv_camera");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this,
                                                                 camera_name_);

    // // 根据配置文件设置相机参数
    MV_CHECK_API(CameraReadParameterFromFile(h_camera_,
                                             (char *)camera_config_url.data()),
                 "从文件中指定参数");

    // 声明 parameters 相机参数
    declareCameraParameters();

    // 获取相机配置文件路径
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "camera info url error! : %s",
                  camera_info_url.c_str());
    }

    // 添加 修改参数 的回调函数
    params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &MVCameraNode::cameraParametersCallback, this, std::placeholders::_1));

    init_thread();
    // 结束
    // rclcpp::shutdown();
  }

  ~MVCameraNode() override {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }

    if (h_camera_ > 0) {
      CameraUnInit(h_camera_);
    }

    RCLCPP_INFO(this->get_logger(), "MindVision Node destoryed!");
  }

private:
  void init_thread() {
    capture_thread_ = std::thread([this]() -> void {
      RCLCPP_INFO(this->get_logger(), "init Publish thread!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        // 获取图片
        int status =
            CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000);

        if (status != CAMERA_STATUS_SUCCESS) {
          RCLCPP_WARN(this->get_logger(), "get Image error: %d", status);
          continue;
        }
        CameraImageProcess(h_camera_, pby_buffer_, image_msg_.data.data(),
                           &s_frame_info_);
        camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
        image_msg_.height = s_frame_info_.iHeight;
        image_msg_.width = s_frame_info_.iWidth;
        image_msg_.step = s_frame_info_.iWidth * 3;
        image_msg_.data.resize(s_frame_info_.iWidth * s_frame_info_.iHeight *
                               3);

        camera_pub_.publish(image_msg_, camera_info_msg_);

        // RCLCPP_INFO(this->get_logger(), "publish image!");

        // 释放
        CameraReleaseImageBuffer(h_camera_, pby_buffer_);

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
      }
    });
  }

  void declareCameraParameters() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    // 对于CMOS传感器，其曝光的单位是按照行来计算的
    double exposure_line_time;
    CameraGetExposureLineTime(h_camera_, &exposure_line_time);
    param_desc.integer_range[0].from_value =
        t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time;
    param_desc.integer_range[0].to_value =
        t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time;

    double exposure_time; // 当前曝光时间
    CameraGetExposureTime(h_camera_, &exposure_time);
    exposure_time =
        this->declare_parameter("exposure_time", exposure_time, param_desc);
    CameraSetExposureTime(h_camera_, exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time = %lf", exposure_time);

    // Analog gain
    param_desc.description = "Analog gain";
    param_desc.integer_range[0].from_value =
        t_capability_.sExposeDesc.uiAnalogGainMin;
    param_desc.integer_range[0].to_value =
        t_capability_.sExposeDesc.uiAnalogGainMax;
    int analog_gain;
    CameraGetAnalogGain(h_camera_, &analog_gain);
    analog_gain =
        this->declare_parameter("analog_gain", analog_gain, param_desc);
    CameraSetAnalogGain(h_camera_, analog_gain);
    RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);

    // RGB Gain
    // Get default value
    CameraGetGain(h_camera_, &r_gain, &g_gain, &b_gain);
    // R Gain
    param_desc.integer_range[0].from_value =
        t_capability_.sRgbGainRange.iRGainMin;
    param_desc.integer_range[0].to_value =
        t_capability_.sRgbGainRange.iRGainMax;
    r_gain = this->declare_parameter("rgb_gain.r", r_gain, param_desc);
    // G Gain
    param_desc.integer_range[0].from_value =
        t_capability_.sRgbGainRange.iGGainMin;
    param_desc.integer_range[0].to_value =
        t_capability_.sRgbGainRange.iGGainMax;
    g_gain = this->declare_parameter("rgb_gain.g", g_gain, param_desc);
    // B Gain
    param_desc.integer_range[0].from_value =
        t_capability_.sRgbGainRange.iBGainMin;
    param_desc.integer_range[0].to_value =
        t_capability_.sRgbGainRange.iBGainMax;
    b_gain = this->declare_parameter("rgb_gain.b", b_gain, param_desc);
    // Set gain
    CameraSetGain(h_camera_, r_gain, g_gain, b_gain);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: R = %d", r_gain);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: G = %d", g_gain);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: B = %d", b_gain);

    // Saturation
    param_desc.description = "Saturation";
    param_desc.integer_range[0].from_value =
        t_capability_.sSaturationRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sSaturationRange.iMax;
    int saturation;
    CameraGetSaturation(h_camera_, &saturation);
    saturation = this->declare_parameter("saturation", saturation, param_desc);
    CameraSetSaturation(h_camera_, saturation);
    RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

    // Gamma
    param_desc.integer_range[0].from_value = t_capability_.sGammaRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sGammaRange.iMax;
    int gamma;
    CameraGetGamma(h_camera_, &gamma);
    gamma = this->declare_parameter("gamma", gamma, param_desc);
    CameraSetGamma(h_camera_, gamma);
    RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);
  }

  rcl_interfaces::msg::SetParametersResult
  cameraParametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    RCLCPP_INFO(this->get_logger(), "set paramters!");
    for (const auto &param : parameters) {
      if (param.get_name() == "exposure_time") { // 曝光时间
        double exposure_time = param.as_double();
        int status = CameraSetExposureTime(h_camera_, exposure_time);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failer to set exposure time: " + std::to_string(exposure_time) +
              ", status = " + std::to_string(status);
        }
      } else if (param.get_name() ==
                 "analog_gain") { // 设置相机的图像模拟增益值
        int analog_gain = param.as_int();
        int status = CameraSetAnalogGain(h_camera_, analog_gain);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failer to set analog_gain: " + std::to_string(analog_gain) +
              ", status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.r") { // 设置图像的数字增益
        r_gain = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain, g_gain, b_gain);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failer to set r_gain: " + std::to_string(r_gain) +
                          ", status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.g") { // 设置图像的数字增益
        g_gain = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain, g_gain, b_gain);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failer to set g_gain: " + std::to_string(g_gain) +
                          ", status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.b") { // 设置图像的数字增益
        b_gain = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain, g_gain, b_gain);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failer to set b_gain: " + std::to_string(b_gain) +
                          ", status = " + std::to_string(status);
        }
      } else if (param.get_name() == "saturation") { // 图像处理的饱和度
        int saturation = param.as_int();
        int status = CameraSetSaturation(h_camera_, saturation);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason =
              "Failer to set saturation: " + std::to_string(saturation) +
              ", status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gamma") {
        int gamma = param.as_int();
        int status = CameraSetGamma(h_camera_, gamma);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failer to set gamma: " + std::to_string(gamma) +
                          ", status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }

    return result;
  }

  int h_camera_; // 相机句柄
  uint8_t *pby_buffer_;
  tSdkCameraCapbility t_capability_; // 设备描述信息
  tSdkFrameHead s_frame_info_;       // 图像帧头信息

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  // RGB gain
  int r_gain, g_gain, b_gain;

  bool flip_image_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  rmw_qos_profile_t qos_least = rmw_qos_profile_sensor_data; // 传感器数据
  // auto a = RELIABILITY_QOS_POLICY;
  rmw_qos_profile_t qos_buffer = {RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                  5,
                                  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                  RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                  RMW_QOS_DEADLINE_DEFAULT,
                                  RMW_QOS_LIFESPAN_DEFAULT,
                                  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                  false};
};

} // namespace mindvision_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mindvision_camera::MVCameraNode)