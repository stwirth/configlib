#pragma once
#include <configlib/exceptions.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

namespace configlib {
class Parameter
{

public:

  inline Parameter(const std::string &name) : name_(name) {
  }

  inline std::string name() const {
    return name_;
  }

  template<typename T>
  void set(const T& value) {
    value_ = XmlRpc::XmlRpcValue(value);
  }

  template<typename T>
  T as() const {
    if (!value_.valid()) {
      throw NoValueException(name_);
    }
    try {
      // couldn't find a way without this ugly cast
      return static_cast<T>(const_cast<Parameter*>(this)->value_);
    } catch (const XmlRpc::XmlRpcException&) {
      throw TypeMismatchException(name_);
    }
  }

  template<typename T>
  bool isType() const {
    // For C++17 std::variant this will change to std::holds_alternative
    try {
      as<T>();
      return true;
    } catch (const TypeMismatchException&) {
    } catch (const NoValueException&) {
    }
    return false;
  }

private:

  std::string name_;

  // TODO switch from XmlRpcValue to std::variant in C++17
  // (with std::monotype as first alternative to have an "empty" value)
  XmlRpc::XmlRpcValue value_;
  XmlRpc::XmlRpcValue min_value_;
  XmlRpc::XmlRpcValue max_value_;

};
} // namespace configlib

