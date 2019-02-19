#include <configlib/config.h>
#include <configlib/config_server_ros.h>
#include <ros/ros.h>

class FancyAlgorithm
{
public:

  FancyAlgorithm() : cfg_(new configlib::Config())
  {
    cfg_->add("int_param").set(42);
    cfg_->add("double_param").set(4.2);
    cfg_->add("bool_param").set(true);
    cfg_->add("str_param").set("Hello World!");
  }

  void run()
  {
  }

  configlib::Config::Ptr getConfig()
  {
    return cfg_;
  }

private:

  configlib::Config::Ptr cfg_;

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "configlib_example");

  // we set the logger level to debug for demonstration purpose
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::console::notifyLoggerLevelsChanged();

  FancyAlgorithm algo;
  ros::NodeHandle nh("~");
  configlib::ConfigServerROS cs(nh, algo.getConfig());
  cs.start();

  ros::Rate rate(1.0);
  while (ros::ok()) {
    algo.run();
    rate.sleep();
  }

  return 0;
}
