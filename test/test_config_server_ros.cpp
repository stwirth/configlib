#include <configlib/config.h>
#include <configlib/config_server_ros.h>

#include <gtest/gtest.h>

namespace configlib {
namespace {

TEST(ConfigServerROS, readFromROSParameterServer)
{
  ros::NodeHandle nh("~");
  nh.setParam("int_param", 42);
  nh.setParam("double_param", 123.456);
  nh.setParam("bool_param", true);
  nh.setParam("str_param", "Hello World!");

  Config::Ptr cfg(new Config());
  cfg->add("int_param");
  cfg->add("double_param");
  cfg->add("bool_param");
  cfg->add("str_param");

  ConfigServerROS cs(nh, cfg);
  EXPECT_NO_THROW(cs.start());

  EXPECT_EQ(42, cfg->get("int_param").as<int>());
  EXPECT_FLOAT_EQ(123.456, cfg->get("double_param").as<double>());
  EXPECT_EQ(true, cfg->get("bool_param").as<bool>());
  EXPECT_EQ("Hello World!", cfg->get("str_param").as<std::string>());
}

TEST(ConfigServerROS, invalidROSNameThrows)
{
  Config::Ptr cfg(new Config());
  cfg->add("int param");

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);
  EXPECT_THROW(cs.start(), InvalidNameException);
}

} // anonymous namespace
} // namespace configlib

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_config_server_ros");
  return RUN_ALL_TESTS();
}

