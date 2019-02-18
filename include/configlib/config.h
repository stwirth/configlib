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

  std::vector<Parameter>& parameters();

  const std::vector<Parameter>& parameters() const;

  // for use with std::lock_guard
  void lock();

  void unlock();

private:

  std::vector<Parameter> parameters_;

  // for O(1) access
  std::unordered_map<std::string, int> parameter_index_;
  std::mutex parameters_mutex_;

};

} // namespace configlib

