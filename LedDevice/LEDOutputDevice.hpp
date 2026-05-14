#pragma once
#include <Device/Protocol/DeviceInterface.hpp>
#include <Device/Protocol/DeviceSettings.hpp>
#include <Device/Protocol/ProtocolFactoryInterface.hpp>
#include <Device/Protocol/ProtocolSettingsWidget.hpp>

#include <Gfx/GfxDevice.hpp>
#include <Gfx/SharedOutputSettings.hpp>

#include <QLineEdit>

namespace Gfx
{

class gfx_protocol_base;
class LedOutputProtocolFactory final : public Gfx::SharedOutputProtocolFactory
{
  SCORE_CONCRETE("83416828-8acf-4d32-be83-d4ad223a89a0")
public:
  QString prettyName() const noexcept override;
  QUrl manual() const noexcept override;
  QString category() const noexcept override;

  Device::DeviceInterface* makeDevice(
      const Device::DeviceSettings& settings, const Explorer::DeviceDocumentPlugin& doc,
      const score::DocumentContext& ctx) override;
  const Device::DeviceSettings& defaultSettings() const noexcept override;

  Device::ProtocolSettingsWidget* makeSettingsWidget() override;
};

}
