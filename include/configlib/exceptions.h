#pragma once

#include <stdexcept>

namespace configlib {

class Exception : public std::runtime_error
{
public:
  Exception(const std::string &what) : std::runtime_error(what)
  {}
};

class NoValueException : public Exception
{
public:
  NoValueException() : Exception("parameter has no value")
  {}

  NoValueException(const std::string &name) :
    Exception(std::string("parameter '") + name + "' has no value")
  {}
};

class UnknownParameterException : public Exception
{
public:
  UnknownParameterException(const std::string &name) :
    Exception(std::string("unknown parameter '") + name + "'")
  {}
};

class TypeMismatchException : public Exception
{
public:
  TypeMismatchException() : Exception("type mismatch")
  {}

  TypeMismatchException(const std::string &name) :
    Exception(std::string("type mismatch for parameter '") + name + "'")
  {}

};

} // namespace configlib

