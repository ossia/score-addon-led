#pragma once
#include <score/tools/std/StringHash.hpp>

#include <verdigris>

namespace Led
{
struct SpecificSettings
{
    QString device;
    int num_pixels{12};
    int speed{100};
};
}

Q_DECLARE_METATYPE(Led::SpecificSettings)
W_REGISTER_ARGTYPE(Led::SpecificSettings)
