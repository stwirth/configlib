#pragma once

#include <configlib/parameter.h>
#include <configlib/config.h>

#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/ConfigDescription.h>

namespace configlib
{

namespace dynamic_reconfigure_conversions
{

dynamic_reconfigure::ParamDescription createParamDescription(const Parameter &param);
dynamic_reconfigure::ConfigDescription createConfigDescription(const Config &config);

} // namespace dynamic_reconfigure_conversions

} // namespace configlib

