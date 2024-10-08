#include "ProtocolSettingsWidget.hpp"

#include "ProtocolFactory.hpp"
#include "SpecificSettings.hpp"

#include <Library/LibrarySettings.hpp>
#include <State/Widgets/AddressFragmentLineEdit.hpp>

#include <score/application/ApplicationContext.hpp>
#include <score/model/tree/TreeNodeItemModel.hpp>
#include <score/tools/FindStringInFile.hpp>

#include <ossia/detail/config.hpp>

#include <ossia/detail/flat_map.hpp>

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDirIterator>
#include <QFormLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QVariant>

#include <wobjectimpl.h>

W_OBJECT_IMPL(Led::ProtocolSettingsWidget)

namespace Led
{
ProtocolSettingsWidget::ProtocolSettingsWidget(QWidget* parent)
    : Device::ProtocolSettingsWidget(parent)
{
  m_deviceNameEdit = new State::AddressFragmentLineEdit{this};
  m_deviceNameEdit->setText("Led");

  m_spiDevice = new QLineEdit{this};
  m_spiDevice->setText("/dev/spidev0.0");

  m_pixels = new QSpinBox{this};
  m_pixels->setRange(1, 65535);

  m_format = new QComboBox{this};
  m_format->addItems({"GRB", "RGB"});

  m_fps = new QDoubleSpinBox{this};
  m_fps->setRange(0.001, 1000);
  m_fps->setValue(60.);

  auto layout = new QFormLayout;
  layout->addRow(tr("Name"), m_deviceNameEdit);
  layout->addRow(tr("Device"), m_spiDevice);
  layout->addRow(tr("Pixels"), m_pixels);
  layout->addRow(tr("Format"), m_format);
  layout->addRow(tr("FPS"), m_fps);

  setLayout(layout);
}

ProtocolSettingsWidget::~ProtocolSettingsWidget() { }

Device::DeviceSettings ProtocolSettingsWidget::getSettings() const
{
  Device::DeviceSettings s;
  s.name = m_deviceNameEdit->text();
  s.protocol = ProtocolFactory::static_concreteKey();

  SpecificSettings settings{};
  settings.device = this->m_spiDevice->text();
  settings.num_pixels = this->m_pixels->value();
  settings.format
      = static_cast<NeoPixelsFormat>(this->m_format->currentIndex());
  settings.fps = this->m_fps->value();
  s.deviceSpecificSettings = QVariant::fromValue(settings);

  return s;
}

void ProtocolSettingsWidget::setSettings(
    const Device::DeviceSettings& settings)
{
  m_deviceNameEdit->setText(settings.name);
  const auto& specif
      = settings.deviceSpecificSettings.value<SpecificSettings>();
  m_spiDevice->setText(specif.device);
  m_pixels->setValue(specif.num_pixels);
  m_format->setCurrentIndex(static_cast<int>(specif.format));
  m_fps->setValue(specif.fps);
}
}
