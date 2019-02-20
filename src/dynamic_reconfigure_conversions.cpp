#include <configlib/config_server_ros.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

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

void addMinMaxDefaultConfig(const Parameter &param, dynamic_reconfigure::ConfigDescription &config_desc)
{
  // default is the current value (the one on startup)
  if (param.isType<int>()) {

    dynamic_reconfigure::IntParameter int_param;
    int_param.name = param.name();

    int_param.value = std::numeric_limits<int>::lowest();
    config_desc.min.ints.push_back(int_param);

    int_param.value = std::numeric_limits<int>::max();
    config_desc.max.ints.push_back(int_param);

    int_param.value = param.as<int>();
    config_desc.dflt.ints.push_back(int_param);

  } else if (param.isType<double>()) {

    dynamic_reconfigure::DoubleParameter double_param;
    double_param.name = param.name();

    double_param.value = std::numeric_limits<double>::lowest();
    config_desc.min.doubles.push_back(double_param);

    double_param.value = std::numeric_limits<double>::max();
    config_desc.max.doubles.push_back(double_param);

    double_param.value = param.as<double>();
    config_desc.dflt.doubles.push_back(double_param);

  } else if (param.isType<bool>()) {

    dynamic_reconfigure::BoolParameter bool_param;
    bool_param.name = param.name();

    bool_param.value = false;
    config_desc.min.bools.push_back(bool_param);

    bool_param.value = true;
    config_desc.max.bools.push_back(bool_param);

    bool_param.value = param.as<bool>();
    config_desc.dflt.bools.push_back(bool_param);

  } else if (param.isType<std::string>()) {

    dynamic_reconfigure::StrParameter str_param;
    str_param.name = param.name();
    str_param.value = param.as<std::string>();
    config_desc.dflt.strs.push_back(str_param);

  } else {
    throw IncompatibleTypeException();
  }
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
      addMinMaxDefaultConfig(param, config_desc);
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

