#pragma once

namespace configlib
{

class ConfigServerROS
{
public:

  void ConfigServerROS(const ros::NodeHandle &nh,
      const Config::Ptr &cfg) : nh_(nh), cfg_(cfg)
  {}

private:

  ros::NodeHandle nh_;
  Config::Ptr cfg_;

};

