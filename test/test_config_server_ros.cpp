#include <configlib/config.h>
#include <configlib/config_server_ros.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <gtest/gtest.h>

namespace configlib {
namespace {

dynamic_reconfigure::Reconfigure::Request createTestDynamicReconfigureRequest()
{
  dynamic_reconfigure::Reconfigure::Request req;

  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = 42;
  req.config.ints.push_back(int_param);

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "double_param";
  double_param.value = 123.456;
  req.config.doubles.push_back(double_param);

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "bool_param";
  bool_param.value = true;
  req.config.bools.push_back(bool_param);

  dynamic_reconfigure::StrParameter str_param;
  str_param.name = "str_param";
  str_param.value = "Hello World!";
  req.config.strs.push_back(str_param);

  return req;
}

TEST(ConfigServerROS, valuesAreReadFromROSParameterServer)
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

TEST(ConfigServerROS, startThrowsOnInvalidROSName)
{
  Config::Ptr cfg(new Config());
  cfg->add("invalid param name");

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);
  EXPECT_THROW(cs.start(), InvalidNameException);
}

TEST(ConfigServerROS, startThrowsOnTypeMismatch)
{
  Config::Ptr cfg(new Config());
  // define integer parameter
  cfg->add("param").set(42);

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);

  // set parameter as string on ROS parameter server
  nh.setParam("param", "Hello World!");
  EXPECT_THROW(cs.start(), TypeMismatchException);
}

/**
 * Helper class to count how many messages have been received.
 */
template<typename T>
struct MessageCounter {
  int count = 0;
  void callback(const typename T::ConstPtr &msg) {
    count++;
  }
};

TEST(ConfigServerROS, setParametersServiceChangesConfig)
{
  Config::Ptr cfg(new Config());
  cfg->add("int_param");
  cfg->add("double_param");
  cfg->add("bool_param");
  cfg->add("str_param");

  ros::NodeHandle nh("~");
  // make sure to clear the ROS parameter server
  // (there may be leftovers from previous tests)
  nh.deleteParam(nh.getNamespace());
  ConfigServerROS cs(nh, cfg);
  cs.start();

  ros::ServiceClient client =
    nh.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");
  dynamic_reconfigure::Reconfigure::Request request = createTestDynamicReconfigureRequest();
  dynamic_reconfigure::Reconfigure::Response response;

  ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));

  // check that parameter_updates are sent
  MessageCounter<dynamic_reconfigure::Config> message_counter;
  int queue_size = 1;
  ros::Subscriber sub = nh.subscribe("parameter_updates", queue_size,
      &MessageCounter<dynamic_reconfigure::Config>::callback, &message_counter);

  EXPECT_TRUE(client.call(request, response));
  EXPECT_EQ(1, message_counter.count);

  EXPECT_EQ(request.config.ints[0].value, cfg->get("int_param").as<int>());
  EXPECT_FLOAT_EQ(request.config.doubles[0].value, cfg->get("double_param").as<double>());
  EXPECT_EQ(request.config.bools[0].value, cfg->get("bool_param").as<bool>());
  EXPECT_EQ(request.config.strs[0].value, cfg->get("str_param").as<std::string>());
}

TEST(ConfigServerROS, parameterUpdatesAreSent)
{
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

