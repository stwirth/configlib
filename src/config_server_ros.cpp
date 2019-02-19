#include <configlib/config_server_ros.h>
#include <configlib/dynamic_reconfigure_conversions.h>

namespace configlib
{

ConfigServerROS::ConfigServerROS(const ros::NodeHandle &nh, const Config::Ptr &cfg) :
  nh_(nh), cfg_(cfg)
{
}

void ConfigServerROS::start()
{
  using XmlRpc::XmlRpcValue;
  {
    std::lock_guard<Config> lock(*cfg_);
    for (auto& param : cfg_->parameters()) {
      std::string name = param.name();
      try {
        XmlRpcValue value;
        if (nh_.getParam(name, value)) {
          switch (value.getType()) {
            case XmlRpcValue::TypeInt:
              param.set(static_cast<int>(value));
              break;
            case XmlRpcValue::TypeDouble:
              param.set(static_cast<double>(value));
              break;
            case XmlRpcValue::TypeBoolean:
              param.set(static_cast<bool>(value));
              break;
            case XmlRpcValue::TypeString:
              param.set(static_cast<std::string>(value));
              break;
            default:
              throw UnsupportedTypeException(name);
          }
        }
      } catch (const ros::InvalidNameException &e) {
        throw InvalidNameException(name, e.what());
      }
    }
  }
  int queue_size = 1;
  bool latch = true;
  parameter_updates_pub_ = nh_.advertise<dynamic_reconfigure::Config>(
      "parameter_updates", queue_size, latch);
  parameter_descriptions_pub_ = nh_.advertise<dynamic_reconfigure::ConfigDescription>(
      "parameter_descriptions", queue_size, latch);
  publishParameterDescriptions();
  publishParameterUpdates();
  reconfigure_service_ = nh_.advertiseService("set_parameters",
      &ConfigServerROS::setParameters, this);
}

/**
 * Returns false if one or more parameters could not be set.
 */
bool ConfigServerROS::setParameters(
    dynamic_reconfigure::Reconfigure::Request &req,
    dynamic_reconfigure::Reconfigure::Response &res)
{
  std::vector<std::string> updated_params;
  try {
    for (const auto &int_param : req.config.ints) {
      ROS_DEBUG_STREAM("Setting int parameter '" << int_param.name
          << "' to " << int_param.value);
      cfg_->get(int_param.name).set(int_param.value);
    }
    for (const auto &double_param : req.config.doubles) {
      ROS_DEBUG_STREAM("Setting double parameter '" << double_param.name
          << "' to " << double_param.value);
      cfg_->get(double_param.name).set(double_param.value);
    }
    for (const auto &bool_param : req.config.bools) {
      ROS_DEBUG_STREAM("Setting bool parameter '" << bool_param.name
          << "' to " << bool_param.value);
      // explicit cast needed as bool is encoded as uint8_t in ROS messages
      cfg_->get(bool_param.name).set(static_cast<bool>(bool_param.value));
    }
    for (const auto &str_param : req.config.strs) {
      ROS_DEBUG_STREAM("Setting string parameter '" << str_param.name
          << "' to " << str_param.value);
      cfg_->get(str_param.name).set(str_param.value);
    }
  } catch (const Exception &e) {
    ROS_ERROR_STREAM("Error processing dynamic reconfigure request: " << e.what());
    return false;
  }
  publishParameterUpdates();
  return true;
}

void ConfigServerROS::publishParameterDescriptions()
{
  std::lock_guard<Config> lock(*cfg_);
  dynamic_reconfigure::ConfigDescription config_desc =
    dynamic_reconfigure_conversions::createConfigDescription(*cfg_);
  parameter_descriptions_pub_.publish(config_desc);
}

void ConfigServerROS::publishParameterUpdates()
{
  std::lock_guard<Config> lock(*cfg_);
  dynamic_reconfigure::Config updates;
  for (const auto& param : cfg_->parameters()) {
    const std::string name = param.name();
    if (param.isType<int>()) {
      dynamic_reconfigure::IntParameter int_param;
      int_param.value = param.as<int>();
      updates.ints.push_back(int_param);
      nh_.setParam(name, int_param.value);
    } else if (param.isType<double>()) {
      dynamic_reconfigure::DoubleParameter double_param;
      double_param.value = param.as<double>();
      updates.doubles.push_back(double_param);
      nh_.setParam(name, double_param.value);
    } else if (param.isType<bool>()) {
      dynamic_reconfigure::BoolParameter bool_param;
      bool_param.value = param.as<bool>();
      updates.bools.push_back(bool_param);
      nh_.setParam(name, bool_param.value);
    } else if (param.isType<std::string>()) {
      dynamic_reconfigure::StrParameter str_param;
      str_param.value = param.as<std::string>();
      updates.strs.push_back(str_param);
      nh_.setParam(name, str_param.value);
    } else {
      ROS_WARN_STREAM(
          "Cannot publish parameter updates for parameter '"
          << name << "' (type not supported in ROS dynamic_reconfigure)");
    }
  }
  parameter_updates_pub_.publish(updates);
}

} // namespace configlib

