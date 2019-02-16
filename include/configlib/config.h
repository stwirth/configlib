#pragma once

#include <configlib/parameter.h>

#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>

namespace configlib {

/**
 * Key/Value storage with std::string keys and a variant as value.
 */
class Config
{

public:

  typedef std::shared_ptr<Config> Ptr;

  Parameter& add(const std::string &name);

  Parameter& get(const std::string &name);

private:

  std::unordered_map<std::string, Parameter> parameters_;
  std::mutex parameters_mutex_;

};

} // namespace configlib

