#pragma once
#include <score/tools/std/StringHash.hpp>

#include <verdigris>

namespace Led
{
enum class NeoPixelsFormat
{
  GRB,
  RGB
};

struct SpecificSettings
{
    QString device;
    int num_pixels{12};
    int speed{800};
    NeoPixelsFormat format{NeoPixelsFormat::GRB};
};
}

Q_DECLARE_METATYPE(Led::SpecificSettings)
W_REGISTER_ARGTYPE(Led::SpecificSettings)
