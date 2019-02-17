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
  TypeMismatchException(const std::string &name) :
    Exception(std::string("type mismatch for parameter '") + name + "'")
  {}

};

} // namespace configlib

