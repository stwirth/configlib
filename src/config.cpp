#include <configlib/config.h>
#include <configlib/exceptions.h>

namespace configlib
{

Parameter& Config::add(const std::string &name) {
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  parameters_[name] = Parameter();
  return parameters_[name];
}

Parameter& Config::get(const std::string &name) {
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  auto it = parameters_.find(name);
  if (it == parameters_.end()) {
    throw UnknownParameterException(name);
  }
  return it->second;
}

} // namespace configlib

