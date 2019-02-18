#include <configlib/config.h>
#include <configlib/config_server_ros.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <gtest/gtest.h>

namespace configlib {
namespace {

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

/**
 * Helper class to hold on to a message received by a callback.
 */
template<typename T>
struct MessageReceiver {
  typename T::ConstPtr last_message;
  void callback(const typename T::ConstPtr &msg) {
    ROS_INFO("callback!");
    last_message = msg;
  }
};

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

TEST(ConfigServerROS, parameterDescriptionsArePublishedOnStart)
{
  Config::Ptr cfg(new Config());
  cfg->add("int_param").set(42);

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);

  MessageCounter<dynamic_reconfigure::ConfigDescription> message_counter;
  int queue_size = 1;
  ros::Subscriber sub = nh.subscribe("parameter_descriptions", queue_size,
      &MessageCounter<dynamic_reconfigure::ConfigDescription>::callback, &message_counter);
  message_counter.count = 0;

  cs.start();
  ros::Duration(1).sleep();

  EXPECT_EQ(1, message_counter.count);
}

TEST(ConfigServerROS, parameterDescriptionsHaveCorrectContent)
{
  Config::Ptr cfg(new Config());
  cfg->add("int_param").set(123);
  cfg->add("double_param").set(4.56);
  cfg->add("bool_param").set(true);
  cfg->add("str_param").set("Hello World!");

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);

  MessageReceiver<dynamic_reconfigure::ConfigDescription> message_receiver;
  int queue_size = 1;
  ros::Subscriber sub = nh.subscribe("parameter_descriptions", queue_size,
      &MessageReceiver<dynamic_reconfigure::ConfigDescription>::callback, &message_receiver);

  cs.start();
  ros::Duration(1).sleep();

  ASSERT_TRUE(message_receiver.last_message != 0);
  ASSERT_EQ(1, message_receiver.last_message->groups.size());
  ASSERT_EQ(4, message_receiver.last_message->groups[0].parameters.size());
}


TEST(ConfigServerROS, setParametersServiceChangesConfig)
{
  Config::Ptr cfg(new Config());
  cfg->add("int_param").set(123);
  cfg->add("double_param").set(4.56);
  cfg->add("bool_param").set(true);
  cfg->add("str_param").set("Hello World!");

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);
  cs.start();

  ros::ServiceClient client =
    nh.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");

  dynamic_reconfigure::Reconfigure::Request req;
  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = 987;
  req.config.ints.push_back(int_param);

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "double_param";
  double_param.value = 6.54;
  req.config.doubles.push_back(double_param);

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "bool_param";
  bool_param.value = false;
  req.config.bools.push_back(bool_param);

  dynamic_reconfigure::StrParameter str_param;
  str_param.name = "str_param";
  str_param.value = "Hooray for variants!";
  req.config.strs.push_back(str_param);

  ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
  dynamic_reconfigure::Reconfigure::Response res;
  EXPECT_TRUE(client.call(req, res));
  EXPECT_EQ(req.config.ints[0].value, cfg->get("int_param").as<int>());
  EXPECT_FLOAT_EQ(req.config.doubles[0].value, cfg->get("double_param").as<double>());
  EXPECT_EQ(req.config.bools[0].value, cfg->get("bool_param").as<bool>());
  EXPECT_EQ(req.config.strs[0].value, cfg->get("str_param").as<std::string>());
}

TEST(ConfigServerROS, parameterUpdatesAreSentOnStart)
{
  Config::Ptr cfg(new Config());
  cfg->add("param").set(42);

  ros::NodeHandle nh("~");
  ConfigServerROS cs(nh, cfg);

  MessageCounter<dynamic_reconfigure::Config> message_counter;
  int queue_size = 1;
  ros::Subscriber sub = nh.subscribe("parameter_updates", queue_size,
      &MessageCounter<dynamic_reconfigure::Config>::callback, &message_counter);

  message_counter.count = 0;
  cs.start();
  ros::spinOnce();
  EXPECT_EQ(1, message_counter.count);
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

