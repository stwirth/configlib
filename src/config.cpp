#include <configlib/config.h>
#include <configlib/exceptions.h>

namespace configlib
{

Parameter& Config::add(const std::string &name) {
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  parameters_.push_back(Parameter(name));
  parameter_index_[name] = parameters_.size() - 1;
  return parameters_.back();
}

Parameter& Config::get(const std::string &name) {
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  auto it = parameter_index_.find(name);
  if (it == parameter_index_.end()) {
    throw UnknownParameterException(name);
  }
  return parameters_[it->second];
}

std::vector<Parameter>& Config::parameters() {
  return parameters_;
}

const std::vector<Parameter>& Config::parameters() const {
  return parameters_;
}

void Config::lock() {
  parameters_mutex_.lock();
}

void Config::unlock() {
  parameters_mutex_.unlock();
}


} // namespace configlib

