#include <configlib/config.h>
#include <configlib/config_server_ros.h>

#include <gtest/gtest.h>

namespace configlib {
namespace {

TEST(ConfigServerROS, foo)
{
  ros::NodeHandle nh("~");
  nh.setParam("int_param"

  Config::Ptr cfg(new Config());
  cfg->add("int_param");
  cfg->add("double_param");
  cfg->add("bool_param");
  cfg->add("str_param");

  ConfigServerROS cs(nh, cfg);
  cs.start();
}

} // anonymous namespace
} // namespace configlib

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

