#include "serial_test/serial.hpp"

using MA::Respone;
using MA::Result;
using ReceiveData = MA::Serial::ReceiveData;

Respone MA::Serial::open() {
  Respone res;

  if (!isExist()) {
    return {false, "设备不存在。name= " + serial_name};
  }

  std::string command =
      "echo " + password + " | sudo -S chmod 777 " + serial_name;
  if (system(command.c_str()) < 0) {
    return {false, "串口设备赋权失败。name= " + serial_name +
                       ", password= " + password};
  }

  // ::open 是库函数
  int fd = ::open(serial_name.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    return {false, "串口打开失败。name= " + serial_name};
  }

  if (int2baudrate.find(baudrate) == int2baudrate.end()) {
    return {false, "波特率不存在。baudrate= " + std::to_string(baudrate)};
  }

  struct termios options; // termios为类型名的结构
  tcgetattr(fd, &options);
  // 波特率115200, 8N1
  options.c_iflag = IGNPAR; //输入模式
  options.c_oflag = 0;      //输出模式
  options.c_cflag = int2baudrate.at(baudrate) | CS8 | CLOCAL | CREAD; //控制模式
  options.c_lflag = 0; //本地模式
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  //特殊控制模式
  //当设置为 阻塞模式时生效
  options.c_cc[VTIME] = 0; //最少读取字符数
  options.c_cc[VMIN] = 1;  //超时时间, 单位: 100ms
  // tcflush(fd, TCIFLUSH);             //清除缓冲区
  tcsetattr(fd, TCSANOW, &options); //应用上面的设置

  fd_ = fd;

  return {true, ""};
}

Respone MA::Serial::close() {
  if (fd_ > 0) {
    if (::close(fd_)) {
      return {false, "串口关闭失败。close()"};
    }
  }
  return {true, ""};
}

Result<ReceiveData> MA::Serial::read() {
  /*
  0: 0xff 帧头
  1-4: yaw 偏移角度
  5-8: pitch 偏移角度
  9-12: shoot_speed 射速
  13: color 识别颜色
  14: 0xfe 帧尾
  */
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);
  // select 函数会修改tv的值为剩余时间,所以每次都要重新赋值
  tv.tv_sec = 1; //秒
  tv.tv_usec = 0;
  //利用select函数，在tv时刻内检测串口是否可读
  if (select(fd_ + 1, &fds, NULL, NULL, &tv) < 0) {
    return {"串口读取超时"};
  }
  // 读取
  if (::read(fd_, read_buffer_, 1) != 1) {
    // 串口未掉线时，问题是读取不到帧头
    if (isExist()) {
      return {"帧头读取失败"};
    } else {
      return {"串口掉线"};
    }
  }
  if (read_buffer_[0] != frame_header) {
    // 帧头验证失败
    tcflush(fd_, TCIFLUSH); //清除缓冲区
    return {"帧头校验失败"};
  }
  if (::read(fd_, read_buffer_ + 1, 14) != 14) {
    // 读取剩余内容失败
    return {"内容读取失败"};
  }
  if (read_buffer_[14] != frame_tail) {
    // 帧尾验证失败
    tcflush(fd_, TCIFLUSH); //清除缓冲区
    return {"帧尾校验失败"};
  }

  // 赋值
  for (int i = 0; i < 4; i++) {
    read_yaw.uchars[i] = read_buffer_[1 + i];
    read_pitch.uchars[i] = read_buffer_[5 + i];
    shoot_speed.uchars[i] = read_buffer_[9 + i];
  }
  updateShootSpeed(shoot_speed.f);
  // 避免串口颜色位不是0,1
  if (read_buffer_[13] == 0x00) {
    read_color = BLUE;
  } else if (read_buffer_[13] == 0x01) {
    read_color = RED;
  } else {
  }

  return {ReceiveData{read_yaw.f, read_pitch.f, last_shoot_speed, read_color}};
}

Respone MA::Serial::write(SendData send_data) {
  /*发送数据报
    帧头: 0;
    有无目标: 1;
    yaw轴角度: 2-5;
    pitch轴角度: 6-9;
    递增数据位: 10;
    帧尾: 11;
    */
  send_yaw.f = send_data.send_yaw;
  send_pitch.f = send_data.send_pitch;
  //帧头、帧尾
  send_buffer_[0] = frame_header;
  send_buffer_[11] = frame_tail;

  send_buffer_[1] = send_data.goal;

  for (int i = 0; i < 4; i++) {
    send_buffer_[i + 2] = send_yaw.uchars[i];
    send_buffer_[i + 6] = send_pitch.uchars[i];
  }

  send_buffer_[10] = loss;
  loss = (loss + 1) % 0xff;

  if (::write(fd_, send_buffer_, 12) != 12) {
    return {false, "串口发送失败"};
  }

  return {true, ""};
}

bool MA::Serial::isOpen() { return fd_ > 0; }

bool MA::Serial::isExist() { return access(serial_name.c_str(), 0) == 0; }

void MA::Serial::updateShootSpeed(float new_speed) {
  if (new_speed <= 0) {
    return;
  }
  double a = 0.99;
  new_speed = a * last_shoot_speed + (1 - a) * new_speed;
  last_shoot_speed = new_speed;
  return;
}