#pragma once
#include <Device/Protocol/DeviceSettings.hpp>
#include <Device/Protocol/ProtocolSettingsWidget.hpp>

#include <Led/SpecificSettings.hpp>

#include <verdigris>

class QLineEdit;
class QSpinBox;
class QDoubleSpinBox;
class QTableWidget;
class QPushButton;

namespace Led
{
class ProtocolSettingsWidget final : public Device::ProtocolSettingsWidget
{
  W_OBJECT(ProtocolSettingsWidget)

public:
  explicit ProtocolSettingsWidget(QWidget* parent = nullptr);
  virtual ~ProtocolSettingsWidget();
  Device::DeviceSettings getSettings() const override;
  void setSettings(const Device::DeviceSettings& settings) override;

private:
  QLineEdit* m_deviceNameEdit{};
  QLineEdit* m_spiDevice{};
  QSpinBox* m_pixels{};
  QComboBox* m_format{};
  QDoubleSpinBox* m_fps{};
};
}
