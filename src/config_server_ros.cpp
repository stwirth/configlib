#include <configlib/config_server_ros.h>

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
  reconfigure_service_ = nh_.advertiseService("set_parameters",
      &ConfigServerROS::setParameters, this);
}

bool ConfigServerROS::setParameters(
    dynamic_reconfigure::Reconfigure::Request &req,
    dynamic_reconfigure::Reconfigure::Response &res)
{
  try {
    for (const auto &int_param : req.config.ints) {
      ROS_INFO_STREAM("Setting int parameter '" << int_param.name
          << "' to " << int_param.value);
      cfg_->get(int_param.name).set(int_param.value);
    }
    for (const auto &double_param : req.config.doubles) {
      ROS_INFO_STREAM("Setting double parameter '" << double_param.name
          << "' to " << double_param.value);
      cfg_->get(double_param.name).set(double_param.value);
    }
    for (const auto &bool_param : req.config.bools) {
      ROS_INFO_STREAM("Setting bool parameter '" << bool_param.name
          << "' to " << bool_param.value);
      // explicit cast needed as bool is encoded as uint8_t in ROS messages
      cfg_->get(bool_param.name).set(static_cast<bool>(bool_param.value));
    }
    for (const auto &str_param : req.config.strs) {
      ROS_INFO_STREAM("Setting string parameter '" << str_param.name
          << "' to " << str_param.value);
      cfg_->get(str_param.name).set(str_param.value);
    }
  } catch (const Exception &e) {
    ROS_ERROR_STREAM("Error processing dynamic reconfigure request: " << e.what());
    return false;
  }
  return true;
}

} // namespace configlib

