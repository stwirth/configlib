#include <configlib/config_server_ros.h>

namespace configlib
{

ConfigServerROS::ConfigServerROS(const ros::NodeHandle &nh, const Config::Ptr &cfg) :
  nh_(nh), cfg_(cfg)
{}

void ConfigServerROS::start()
{
  using XmlRpc::XmlRpcValue;
  cfg_->lock();
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
  cfg_->unlock();
}

}

