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
    XmlRpc::XmlRpcValue new_value(value);
    if (value_.valid() && value_.getType() != new_value.getType()) {
      throw TypeMismatchException();
    }
    value_ = new_value;
  }

  template<typename T>
  T as() { // TODO make const
    if (!value_.valid()) {
      throw NoValueException();
    }
    try {
      return static_cast<T>(value_);
    } catch (const XmlRpc::XmlRpcException&) {
      throw TypeMismatchException();
    }
  }

private:

  std::string name_;
  XmlRpc::XmlRpcValue value_;

};
} // namespace configlib

