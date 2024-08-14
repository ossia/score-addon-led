#pragma once
#include <halp/controls.hpp>
#include <halp/meta.hpp>
#include <rnd/random.hpp>

#include <algorithm>
#include <vector>

namespace Led
{
struct led_stripe_port
{
  halp_flags(buffer, led_stripe);
  std::vector<uint8_t> value; // In string ?
};

struct LedFX
{
  halp_meta(name, "LedFX")
  halp_meta(c_name, "LedFX")
  halp_meta(category, "UI")
  halp_meta(author, "Jean-Michaël Celerier")
  halp_meta(description, "LedFX")
  halp_meta(uuid, "62bc0879-0b3d-436f-bdd6-7d56db3c2a2f")

  struct
  {
  } inputs;

  struct
  {
    struct
    {
      halp_flags(buffer, led_stripe);
      std::vector<uint8_t> value; // In string ?
    } stripe;
  } outputs;

  void operator()()
  {
    outputs.stripe.value.resize(12 * 3);
    for (int i = 0; i < 12 * 3; i++)
      outputs.stripe.value[i] = r();
  }

  std::random_device dev;
  rnd::pcg r{dev};
};
}
