#include <configlib/config_server_ros.h>

namespace configlib
{

namespace dynamic_reconfigure_conversions
{

class IncompatibleTypeException : public std::runtime_error
{
public:
  IncompatibleTypeException() : std::runtime_error("IncompatibleTypeException")
  {}
};

dynamic_reconfigure::ParamDescription createParamDescription(const Parameter &param)
{
  dynamic_reconfigure::ParamDescription desc;
  desc.name = param.name();
  if (param.isType<int>()) {
    desc.type = "int";
  } else if (param.isType<double>()) {
    desc.type = "double";
  } else if (param.isType<bool>()) {
    desc.type = "bool";
  } else if (param.isType<std::string>()) {
    desc.type = "str";
  } else {
    throw IncompatibleTypeException();
  }
  return desc;
}

dynamic_reconfigure::ConfigDescription createConfigDescription(const Config &cfg)
{
  dynamic_reconfigure::ConfigDescription config_desc;
  config_desc.groups.resize(1);
  config_desc.groups[0].name = "Default";
  for (const auto& param : cfg.parameters()) {
    try {
      dynamic_reconfigure::ParamDescription param_desc = createParamDescription(param);
      config_desc.groups[0].parameters.push_back(param_desc);
    } catch (const IncompatibleTypeException&) {
      ROS_WARN_STREAM(
          "Cannot create parameter description for parameter '"
          << param.name() << "' (type not set or not supported in ROS dynamic_reconfigure)");
    }
  }
  return config_desc;
}

} // namespace dynamic_reconfigure_conversions

} // namespace configlib

