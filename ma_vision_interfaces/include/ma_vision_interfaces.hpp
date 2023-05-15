#ifndef _MA_VISION_INTERFACES_HPP_
#define _MA_VISION_INTERFACES_HPP_


#include <string>

namespace MA {


// 泛型传指针或者内部用指针存，可能更好一点
template<class T>
struct Result{
  T res;
  bool success;
  std::string reason;

  Result<T>(bool success, std::string reason, T res)
  :success(success), reason(reason), res(res)
  {}

  Result<T>(std::string reason)
  :success(false), reason(reason)
  {}

  Result<T>(T res)
  :success(true), reason(""), res(res)
  {}
};


struct Respone{
  bool success;
  std::string reason;

  Respone() = default;

  Respone(bool success, std::string reason)
  :success(success), reason(reason)
  {}
};



enum DetectColor{BLUE = 0, RED = 1};

}
#endif 