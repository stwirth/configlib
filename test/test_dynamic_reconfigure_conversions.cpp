#include <configlib/dynamic_reconfigure_conversions.h>
#include <configlib/parameter.h>
#include <configlib/config.h>

#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <gtest/gtest.h>

namespace configlib {
namespace dynamic_reconfigure_conversions {
namespace {

TEST(DynamicReconfigureConversions, createParamDescriptionInt)
{
  Parameter param("param");
  param.set(42);

  dynamic_reconfigure::ParamDescription param_desc;
  ASSERT_NO_THROW(param_desc = createParamDescription(param));

  EXPECT_EQ("param", param_desc.name);
  EXPECT_EQ("int", param_desc.type);
}

TEST(DynamicReconfigureConversions, createParamDescriptionDouble)
{
  Parameter param("param");
  param.set(3.1415);

  dynamic_reconfigure::ParamDescription param_desc;
  ASSERT_NO_THROW(param_desc = createParamDescription(param));

  EXPECT_EQ("param", param_desc.name);
  EXPECT_EQ("double", param_desc.type);
}

TEST(DynamicReconfigureConversions, createParamDescriptionBool)
{
  Parameter param("param");
  param.set(true);

  dynamic_reconfigure::ParamDescription param_desc;
  ASSERT_NO_THROW(param_desc = createParamDescription(param));

  EXPECT_EQ("param", param_desc.name);
  EXPECT_EQ("bool", param_desc.type);
}

TEST(DynamicReconfigureConversions, createParamDescriptionString)
{
  Parameter param("param");
  param.set("Hello World!");

  dynamic_reconfigure::ParamDescription param_desc;
  ASSERT_NO_THROW(param_desc = createParamDescription(param));

  EXPECT_EQ("param", param_desc.name);
  EXPECT_EQ("str", param_desc.type);
}

TEST(DynamicReconfigureConversions, createConfigDescription)
{
  Config cfg;
  cfg.add("int_param").set(123);
  cfg.add("double_param").set(4.56);
  cfg.add("bool_param").set(true);
  cfg.add("str_param").set("Hello World!");

  dynamic_reconfigure::ConfigDescription config_desc =
    createConfigDescription(cfg);

  ASSERT_EQ(1, config_desc.groups.size());
  ASSERT_EQ(4, config_desc.groups[0].parameters.size());
  // This is a bit over-constraining, the order of the parameters in the message should not matter.
  // However, this is simple to do and may be a nice way of ordering parameters in the UI.
  EXPECT_EQ("int_param", config_desc.groups[0].parameters[0].name);
  EXPECT_EQ("int", config_desc.groups[0].parameters[0].type);
  EXPECT_EQ("double_param", config_desc.groups[0].parameters[1].name);
  EXPECT_EQ("double", config_desc.groups[0].parameters[1].type);
  EXPECT_EQ("bool_param", config_desc.groups[0].parameters[2].name);
  EXPECT_EQ("bool", config_desc.groups[0].parameters[2].type);
  EXPECT_EQ("str_param", config_desc.groups[0].parameters[3].name);
  EXPECT_EQ("str", config_desc.groups[0].parameters[3].type);
}

} // anonymous namespace
} // namespace dynamic_reconfigure_conversions
} // namespace configlib

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

