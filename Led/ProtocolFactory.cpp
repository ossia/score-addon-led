#include "ProtocolFactory.hpp"

#include "Device.hpp"
#include "ProtocolSettingsWidget.hpp"
#include "SpecificSettings.hpp"

#include <State/Widgets/AddressFragmentLineEdit.hpp>

#include <score/application/ApplicationContext.hpp>
#include <score/widgets/SignalUtils.hpp>

#include <ossia/detail/config.hpp>

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QObject>
#include <QUrl>

namespace Led
{

QString ProtocolFactory::prettyName() const noexcept
{
  return QObject::tr("NeoPixel LEDs");
}

QString ProtocolFactory::category() const noexcept
{
  return StandardCategories::lights;
}

QUrl ProtocolFactory::manual() const noexcept
{
  return QUrl("https://ossia.io/score-docs/devices/led-device.html");
}

Device::DeviceEnumerators
ProtocolFactory::getEnumerators(const score::DocumentContext& ctx) const
{
  return {};
}

Device::DeviceInterface* ProtocolFactory::makeDevice(
    const Device::DeviceSettings& settings,
    const Explorer::DeviceDocumentPlugin& plugin,
    const score::DocumentContext& ctx)
{
  return new Led::DeviceImplementation{settings, plugin, ctx};
}

const Device::DeviceSettings& ProtocolFactory::defaultSettings() const noexcept
{
  static const Device::DeviceSettings& settings = [&]
  {
    Device::DeviceSettings s;
    s.protocol = concreteKey();
    s.name = "Led";
    SpecificSettings settings;
    settings.device = "/dev/spidev0.0";
    settings.num_pixels = 12;
    settings.speed = 100;
    s.deviceSpecificSettings = QVariant::fromValue(settings);
    return s;
  }();

  return settings;
}

Device::ProtocolSettingsWidget* ProtocolFactory::makeSettingsWidget()
{
  return new ProtocolSettingsWidget;
}

QVariant ProtocolFactory::makeProtocolSpecificSettings(
    const VisitorVariant& visitor) const
{
  return makeProtocolSpecificSettings_T<SpecificSettings>(visitor);
}

void ProtocolFactory::serializeProtocolSpecificSettings(
    const QVariant& data,
    const VisitorVariant& visitor) const
{
  serializeProtocolSpecificSettings_T<SpecificSettings>(data, visitor);
}

bool ProtocolFactory::checkCompatibility(
    const Device::DeviceSettings& a,
    const Device::DeviceSettings& b) const noexcept
{
  return true;
}
}
