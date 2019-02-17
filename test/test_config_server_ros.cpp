#include <configlib/config.h>
#include <configlib/config_server_ros.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <gtest/gtest.h>

namespace configlib {
namespace {

dynamic_reconfigure::Config createTestReconfigureConfig()
{
  dynamic_reconfigure::Config config;

  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = 42;
  config.ints.push_back(int_param);

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "double_param";
  double_param.value = 123.456;
  config.doubles.push_back(double_param);

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "bool_param";
  bool_param.value = true;
  config.bools.push_back(bool_param);

  dynamic_reconfigure::StrParameter str_param;
  str_param.name = "str_param";
  str_param.value = "Hello World!";
  config.strs.push_back(str_param);

  return config;
}

TEST(ConfigServerROS, readFromROSParameterServer)
{
  Config::Ptr cfg(new Config());
  cfg->add("int_param");
  cfg->add("double_param");
  cfg->add("bool_param");
  cfg->add("str_param");

  ros::NodeHandle nh("~");
  nh.setParam("int_param", 42);
  nh.setParam("double_param", 123.456);
  nh.setParam("bool_param", true);
  nh.setParam("str_param", "Hello World!");

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

TEST(ConfigServerROS, startThrowsOnTypeMismatchInt)
{
  Config::Ptr cfg(new Config());
  cfg->add("param").set(42);

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);

  nh.setParam("param", 789);
  EXPECT_NO_THROW(cs.start());
  nh.setParam("param", 123.456);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", true);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", "Hello World!");
  EXPECT_THROW(cs.start(), TypeMismatchException);
}

TEST(ConfigServerROS, startThrowsOnTypeMismatchDouble)
{
  Config::Ptr cfg(new Config());
  cfg->add("param").set(123.456);

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);

  nh.setParam("param", 42);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", 789.012);
  EXPECT_NO_THROW(cs.start());
  nh.setParam("param", true);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", "Hello World!");
  EXPECT_THROW(cs.start(), TypeMismatchException);
}

TEST(ConfigServerROS, startThrowsOnTypeMismatchBool)
{
  Config::Ptr cfg(new Config());
  cfg->add("param").set(true);

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);

  nh.setParam("param", 42);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", 123.456);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", false);
  EXPECT_NO_THROW(cs.start());
  nh.setParam("param", "Hello World!");
  EXPECT_THROW(cs.start(), TypeMismatchException);
}

TEST(ConfigServerROS, startThrowsOnTypeMismatchString)
{
  Config::Ptr cfg(new Config());
  cfg->add("param").set("Hello World!");

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);

  nh.setParam("param", 42);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", 123.456);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", true);
  EXPECT_THROW(cs.start(), TypeMismatchException);
  nh.setParam("param", "Hello World!");
  EXPECT_NO_THROW(cs.start());
}


TEST(ConfigServerROS, setParametersService)
{
  Config::Ptr cfg(new Config());
  cfg->add("int_param");
  cfg->add("double_param");
  cfg->add("bool_param");
  cfg->add("str_param");

  ros::NodeHandle nh("~");
  // make sure to clearthe param server
  // (there may be leftovers from previous tests)
  nh.deleteParam("int_param");
  nh.deleteParam("double_param");
  nh.deleteParam("bool_param");
  nh.deleteParam("str_param");
  ConfigServerROS cs(nh, cfg);
  cs.start();

  ros::ServiceClient client =
    nh.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");
  dynamic_reconfigure::Reconfigure::Request srv_req;
  srv_req.config = createTestReconfigureConfig();
  dynamic_reconfigure::Reconfigure::Response srv_res;

  ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
  EXPECT_TRUE(client.call(srv_req, srv_res));

  std::cout << "called reconfigure service" << std::endl;

  EXPECT_EQ(srv_req.config.ints[0].value, cfg->get("int_param").as<int>());
  EXPECT_FLOAT_EQ(123.456, cfg->get("double_param").as<double>());
  EXPECT_EQ(true, cfg->get("bool_param").as<bool>());
  EXPECT_EQ("Hello World!", cfg->get("str_param").as<std::string>());
}


} // anonymous namespace
} // namespace configlib

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_config_server_ros");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  return RUN_ALL_TESTS();
}

