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

  bool hasValue() const {
    return value_.valid();
  }

  template<typename T>
  void set(const T& value) {
    value_ = XmlRpc::XmlRpcValue(value);
  }

  template<typename T>
  T as() { // TODO make const
    if (!value_.valid()) {
      throw NoValueException(name_);
    }
    try {
      return static_cast<T>(value_);
    } catch (const XmlRpc::XmlRpcException&) {
      throw TypeMismatchException(name_);
    }
  }

private:

  std::string name_;
  XmlRpc::XmlRpcValue value_;

};
} // namespace configlib

