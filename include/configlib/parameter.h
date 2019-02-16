#pragma once
#include <configlib/exceptions.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

namespace configlib {
class Parameter
{

public:

  template<typename T>
  Parameter& defaultValue(const T& value) {
    set(value_);
    return this;
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
  T as() {
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

  XmlRpc::XmlRpcValue value_;

};
} // namespace configlib

