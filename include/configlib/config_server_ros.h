#pragma once

#include <configlib/config.h>
#include <configlib/exceptions.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <ros/ros.h>

namespace configlib
{

class InvalidNameException : public Exception
{
public:
  InvalidNameException(const std::string &name, const std::string &detail = "") :
    Exception(std::string("invalid parameter name '") + name + "' " + detail)
  {}
};

class UnsupportedTypeException : public Exception
{
public:
  UnsupportedTypeException(const std::string &name) :
    Exception(std::string("unsupported type for parameter '") + name + "' ")
  {}
};

class ConfigServerROS
{
public:

  ConfigServerROS(const ros::NodeHandle &nh, const Config::Ptr &cfg);

  //TODO add version with notify callback?
  // throws if parameter names are not ROS compatible
  void start();

private:

  bool setParameters(dynamic_reconfigure::Reconfigure::Request &req,
    dynamic_reconfigure::Reconfigure::Response &res);

  void publishParameterUpdates();
  void publishParameterDescriptions();

  ros::NodeHandle nh_;
  Config::Ptr cfg_;

  ros::ServiceServer reconfigure_service_;
  ros::Publisher parameter_updates_pub_;
  ros::Publisher parameter_descriptions_pub_;

};

} // namespace configlib

