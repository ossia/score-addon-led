#pragma once
#include <score/tools/std/StringHash.hpp>

#include <verdigris>

namespace Led
{
struct SpecificSettings
{
  int control{1234};
};
}

Q_DECLARE_METATYPE(Led::SpecificSettings)
W_REGISTER_ARGTYPE(Led::SpecificSettings)