#ifndef _MA_SERIAL_HPP_
#define _MA_SERIAL_HPP_

#include <ma_vision_interfaces/ma_vision_interfaces.hpp>

#include <chrono>
#include <fcntl.h> //文件控制选项头文件
#include <mutex>
#include <stdio.h>
#include <termios.h> //linux串口相关的头文件
#include <thread>
#include <unistd.h> //Linux/Unix系统中内置头文件，包含了许多系统服务的函数原型

#include <unordered_map>

namespace MA {

const int SerialBufferMaxSize = 100;
const std::unordered_map<int, int> int2baudrate{
    {57600, B57600},     {115200, B115200},   {230400, B230400},
    {460800, B460800},   {500000, B500000},   {576000, B576000},
    {921600, B921600},   {1000000, B1000000}, {1152000, B1152000},
    {1500000, B1500000}, {2000000, B2000000}, {2500000, B2500000},
    {3000000, B3000000}, {3500000, B3500000}, {4000000, B4000000}};

class Serial {
public:
  struct ReceiveData {
    float yaw;
    float pitch;
    float shoot_speed;
    DetectColor color;

    ReceiveData(){};
    ReceiveData(float yaw, float pitch, float shoot_speed, DetectColor color)
        : yaw(yaw), pitch(pitch), shoot_speed(shoot_speed), color(color) {}
  };

  struct SendData {
    float send_yaw;
    float send_pitch;
    std::uint8_t goal;

    SendData() {
      send_pitch = 0;
      send_pitch = 0;
      goal = 0;
    };
    SendData(double yaw, double pitch) {
      send_yaw = yaw;
      send_pitch = pitch;
      goal = 1;
    }
  };

  union float_uint8_t {
    float f;
    uint8_t uchars[4];
  };

public:
  Serial();

  Respone open();
  Respone close();

  Result<ReceiveData> read();
  Respone write(SendData send_data);

  bool isOpen();
  bool isExist();

  void updateShootSpeed(float new_speed);

public:
  uint8_t frame_header; // 帧头
  uint8_t frame_tail;   // 帧尾
  uint8_t loss = 0;     // 判断是否掉线，循环递增

  std::string serial_name;
  std::string password;
  int baudrate;

  DetectColor read_color;
  float last_shoot_speed = 15;
  float_uint8_t send_yaw, send_pitch;
  float_uint8_t shoot_speed, read_yaw, read_pitch;

  uint8_t send_buffer_[SerialBufferMaxSize]; // 发送数据缓存区
  uint8_t read_buffer_[SerialBufferMaxSize]; // 接受数据缓存区

  int fd_;           //串口句柄
  fd_set fds;        //句柄集合
  struct timeval tv; //时间
};
} // namespace MA

#endif