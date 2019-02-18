#include <configlib/parameter.h>
#include <configlib/exceptions.h>

#include <gtest/gtest.h>

namespace configlib {
namespace {

TEST(Parameter, name)
{
  Parameter param("param");
  EXPECT_EQ("param", param.name());
}

TEST(Parameter, canSetAllTypes)
{
  Parameter param("param");
  param.set(1);
  param.set(2.0);
  param.set(true);
  param.set("Hello World!");
  EXPECT_EQ("Hello World!", param.as<std::string>());
}

TEST(Parameter, asOnEmptyThrows)
{
  Parameter param("param");
  EXPECT_THROW(param.as<int>(), NoValueException);
  EXPECT_THROW(param.as<double>(), NoValueException);
  EXPECT_THROW(param.as<bool>(), NoValueException);
  EXPECT_THROW(param.as<std::string>(), NoValueException);
}

TEST(Parameter, intParam)
{
  Parameter param("param");
  param.set(42);
  EXPECT_NO_THROW(param.as<int>());
  EXPECT_EQ(42, param.as<int>());
  EXPECT_THROW(param.as<double>(), TypeMismatchException);
  EXPECT_THROW(param.as<bool>(), TypeMismatchException);
  EXPECT_THROW(param.as<std::string>(), TypeMismatchException);
}

TEST(Parameter, doubleParam)
{
  Parameter param("param");
  param.set(3.141593);
  ASSERT_NO_THROW(param.as<double>());
  EXPECT_FLOAT_EQ(3.141593, param.as<double>());
  EXPECT_THROW(param.as<int>(), TypeMismatchException);
  EXPECT_THROW(param.as<bool>(), TypeMismatchException);
  EXPECT_THROW(param.as<std::string>(), TypeMismatchException);
}

TEST(Parameter, boolParam)
{
  Parameter param("param");
  param.set(true);
  ASSERT_NO_THROW(param.as<bool>());
  EXPECT_EQ(true, param.as<bool>());
  EXPECT_THROW(param.as<int>(), TypeMismatchException);
  EXPECT_THROW(param.as<double>(), TypeMismatchException);
  EXPECT_THROW(param.as<std::string>(), TypeMismatchException);
}

TEST(Parameter, stringParam)
{
  Parameter param("param");
  param.set("Hello World!");
  EXPECT_THROW(param.as<int>(), TypeMismatchException);
  EXPECT_THROW(param.as<double>(), TypeMismatchException);
  EXPECT_THROW(param.as<bool>(), TypeMismatchException);
  EXPECT_NO_THROW(param.as<std::string>());
}

} // anonymous namespace
} // namespace configlib

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
