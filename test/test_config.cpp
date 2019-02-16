#include <configlib/config.h>

#include <gtest/gtest.h>

namespace configlib {

TEST(Config, setGetEqual)
{
  Config cfg;
  cfg.add("int_param").set(1);
  cfg.add("double_param").set(2.3);
  cfg.add("bool_param").set(true);
  cfg.add("str_param").set("Hello World!");

  auto int_value = cfg.get("int_param").as<int>();
  auto double_value = cfg.get("double_param").as<double>();
  auto bool_value = cfg.get("bool_param").as<bool>();
  auto str_value = cfg.get("str_param").as<std::string>();

  EXPECT_EQ(1, int_value);
  EXPECT_EQ(2.3, double_value);
  EXPECT_EQ(true, bool_value);
  EXPECT_EQ("Hello World!", str_value);
}

TEST(Config, unsetThrowsOnGet)
{
  Config cfg;
  cfg.add("param");
  EXPECT_THROW(cfg.get("param").as<int>(), NoValueException);
}

TEST(Config, unknownKeyThrows)
{
  Config cfg;
  EXPECT_THROW(cfg.get("param"), UnknownParameterException);
}

} // namespace configlib

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

