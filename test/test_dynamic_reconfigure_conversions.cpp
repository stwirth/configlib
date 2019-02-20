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

TEST(DynamicReconfigureConversions, addMinMaxDefaultConfigInt)
{
  Parameter param("param");
  param.set(42);

  dynamic_reconfigure::ConfigDescription config_desc;
  addMinMaxDefaultConfig(param, config_desc);

  ASSERT_EQ(1, config_desc.min.ints.size());
  EXPECT_EQ(std::numeric_limits<int>::lowest(), config_desc.min.ints[0].value);

  ASSERT_EQ(1, config_desc.max.ints.size());
  EXPECT_EQ(std::numeric_limits<int>::max(), config_desc.max.ints[0].value);

  ASSERT_EQ(1, config_desc.dflt.ints.size());
  EXPECT_EQ(42, config_desc.dflt.ints[0].value);
}

TEST(DynamicReconfigureConversions, addMinMaxDefaultConfigDouble)
{
  Parameter param("param");
  param.set(4.2);

  dynamic_reconfigure::ConfigDescription config_desc;
  addMinMaxDefaultConfig(param, config_desc);

  ASSERT_EQ(1, config_desc.min.doubles.size());
  EXPECT_EQ(std::numeric_limits<double>::lowest(), config_desc.min.doubles[0].value);

  ASSERT_EQ(1, config_desc.max.doubles.size());
  EXPECT_EQ(std::numeric_limits<double>::max(), config_desc.max.doubles[0].value);

  ASSERT_EQ(1, config_desc.dflt.doubles.size());
  EXPECT_FLOAT_EQ(4.2, config_desc.dflt.doubles[0].value);
}

TEST(DynamicReconfigureConversions, addMinMaxDefaultConfigBool)
{
  Parameter param("param");
  param.set(true);

  dynamic_reconfigure::ConfigDescription config_desc;
  addMinMaxDefaultConfig(param, config_desc);

  ASSERT_EQ(1, config_desc.min.bools.size());
  EXPECT_EQ(false, config_desc.min.bools[0].value);

  ASSERT_EQ(1, config_desc.max.bools.size());
  EXPECT_EQ(true, config_desc.max.bools[0].value);

  ASSERT_EQ(1, config_desc.dflt.bools.size());
  EXPECT_EQ(true, config_desc.dflt.bools[0].value);
}

TEST(DynamicReconfigureConversions, addMinMaxDefaultConfigString)
{
  Parameter param("param");
  param.set("Hello World!");

  dynamic_reconfigure::ConfigDescription config_desc;
  addMinMaxDefaultConfig(param, config_desc);

  ASSERT_EQ(0, config_desc.min.strs.size());
  ASSERT_EQ(0, config_desc.max.strs.size());
  ASSERT_EQ(1, config_desc.dflt.strs.size());
  EXPECT_EQ("Hello World!", config_desc.dflt.strs[0].value);
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

  EXPECT_EQ(1, config_desc.min.ints.size());
  EXPECT_EQ(1, config_desc.max.ints.size());
  EXPECT_EQ(1, config_desc.dflt.ints.size());

  EXPECT_EQ(1, config_desc.min.doubles.size());
  EXPECT_EQ(1, config_desc.max.doubles.size());
  EXPECT_EQ(1, config_desc.dflt.doubles.size());

  EXPECT_EQ(1, config_desc.min.bools.size());
  EXPECT_EQ(1, config_desc.max.bools.size());
  EXPECT_EQ(1, config_desc.dflt.bools.size());

  // strings don't have min or max
  EXPECT_EQ(0, config_desc.min.strs.size());
  EXPECT_EQ(0, config_desc.max.strs.size());
  EXPECT_EQ(1, config_desc.dflt.strs.size());
}

} // anonymous namespace
} // namespace dynamic_reconfigure_conversions
} // namespace configlib

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

