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

TEST(Parameter, asOnEmptyThrows)
{
  Parameter param("param");
  EXPECT_THROW(param.as<int>(), NoValueException);
  EXPECT_THROW(param.as<double>(), NoValueException);
  EXPECT_THROW(param.as<bool>(), NoValueException);
  EXPECT_THROW(param.as<std::string>(), NoValueException);
}

TEST(Parameter, asInt)
{
  Parameter param("param");
  param.set(42);
  EXPECT_NO_THROW(param.as<int>());
  EXPECT_THROW(param.as<double>(), TypeMismatchException);
  EXPECT_THROW(param.as<bool>(), TypeMismatchException);
  EXPECT_THROW(param.as<std::string>(), TypeMismatchException);
}

TEST(Parameter, asDouble)
{
  Parameter param("param");
  param.set(3.141593);
  EXPECT_THROW(param.as<int>(), TypeMismatchException);
  EXPECT_NO_THROW(param.as<double>());
  EXPECT_THROW(param.as<bool>(), TypeMismatchException);
  EXPECT_THROW(param.as<std::string>(), TypeMismatchException);
}

TEST(Parameter, asBool)
{
  Parameter param("param");
  param.set(true);
  EXPECT_THROW(param.as<int>(), TypeMismatchException);
  EXPECT_THROW(param.as<double>(), TypeMismatchException);
  EXPECT_NO_THROW(param.as<bool>());
  EXPECT_THROW(param.as<std::string>(), TypeMismatchException);
}

TEST(Parameter, asString)
{
  Parameter param("param");
  param.set("Hello World!");
  EXPECT_THROW(param.as<int>(), TypeMismatchException);
  EXPECT_THROW(param.as<double>(), TypeMismatchException);
  EXPECT_THROW(param.as<bool>(), TypeMismatchException);
  EXPECT_NO_THROW(param.as<std::string>());
}

TEST(Parameter, setTwiceInt)
{
  Parameter param("param");
  param.set(42);
  EXPECT_NO_THROW(param.set(10));
  EXPECT_THROW(param.set(3.141502), TypeMismatchException);
  EXPECT_THROW(param.set(true), TypeMismatchException);
  EXPECT_THROW(param.set("Hello World!"), TypeMismatchException);
}

TEST(Parameter, setTwiceDouble)
{
  Parameter param("param");
  param.set(3.1415);
  EXPECT_THROW(param.set(42), TypeMismatchException);
  EXPECT_NO_THROW(param.set(6.283));
  EXPECT_THROW(param.set(true), TypeMismatchException);
  EXPECT_THROW(param.set("Hello World!"), TypeMismatchException);
}

TEST(Parameter, setTwiceBool)
{
  Parameter param("param");
  param.set(true);
  EXPECT_THROW(param.set(42), TypeMismatchException);
  EXPECT_THROW(param.set(6.283), TypeMismatchException);
  EXPECT_NO_THROW(param.set(false));
  EXPECT_THROW(param.set("Hello World!"), TypeMismatchException);
}

TEST(Parameter, setTwiceString)
{
  Parameter param("param");
  param.set("Hello World!");
  EXPECT_THROW(param.set(42), TypeMismatchException);
  EXPECT_THROW(param.set(6.283), TypeMismatchException);
  EXPECT_THROW(param.set(true), TypeMismatchException);
  EXPECT_NO_THROW(param.set("Goodbye!"));
}

} // anonymous namespace
} // namespace configlib

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
