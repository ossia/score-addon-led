#include "score_addon_led.hpp"

#include <score/plugins/FactorySetup.hpp>

#include <Led/ProtocolFactory.hpp>

score_addon_led::score_addon_led() { }

score_addon_led::~score_addon_led() { }

std::vector<score::InterfaceBase*>
score_addon_led::factories(
    const score::ApplicationContext& ctx,
    const score::InterfaceKey& key) const
{
  return instantiate_factories<
      score::ApplicationContext,
      FW<Device::ProtocolFactory, Led::ProtocolFactory>>(ctx, key);
}

#include <score/plugins/PluginInstances.hpp>
SCORE_EXPORT_PLUGIN(score_addon_led)
