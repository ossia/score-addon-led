#include "Device.hpp"

#include "SpecificSettings.hpp"

#include <Explorer/DocumentPlugin/DeviceDocumentPlugin.hpp>

#include <score/document/DocumentContext.hpp>

#include <ossia/detail/timer.hpp>
#include <ossia/network/base/device.hpp>
#include <ossia/network/base/node.hpp>
#include <ossia/network/base/parameter.hpp>
#include <ossia/network/base/protocol.hpp>
#include <ossia/network/common/complex_type.hpp>
#include <ossia/network/context.hpp>
#include <ossia/network/generic/generic_device.hpp>

#include <boost/container/static_vector.hpp>

#include <QDebug>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <wobjectimpl.h>

W_OBJECT_IMPL(Led::DeviceImplementation)

namespace Led
{

// Code adapted from https://github.com/hannescam/NeoSPI
static constexpr std::array<uint8_t, 8> int_to_neopixel(uint8_t val) noexcept
{
  std::array<uint8_t, 8> color;
  color.fill(0xC0);

  for (int cnt = 0; cnt < 8; cnt++)
  {
    if (val & (1 << (7 - cnt)))
      color[cnt] = 0xF8;
  }
  return color;
}

static constexpr std::array<std::array<uint8_t, 8>, 255> int_to_neopixel_table
    = []() constexpr
{
  std::array<std::array<uint8_t, 8>, 255> res;
  for (uint8_t i = 0; i < 255; i++)
    res[i] = int_to_neopixel(i);
  return res;
}();

struct led_protocol : public ossia::net::protocol_base
{
  led_protocol(
      const SpecificSettings& set,
      ossia::net::network_context_ptr ctx)
      : protocol_base{flags{}}
      , m_context{std::move(ctx)}
      , m_timer{m_context->context}
      , m_pixels{set.num_pixels}
      , m_speed{set.speed}
      , m_format{set.format}
  {

    using namespace std::literals;
    m_fd = ::open(set.device.toStdString().c_str(), O_RDWR);
    if (m_fd < 0)
    {
      qDebug() << "Could not open " + set.device;
      return;
    }

    constexpr float frequency = 10;
    m_timer.set_delay(std::chrono::milliseconds{
        static_cast<int>(1000.0f / static_cast<float>(frequency))});
  }

  ~led_protocol()
  {
    ::close(m_fd);
    m_timer.stop();
  }

  void set_device(ossia::net::device_base& dev) override
  {
    m_device = &dev;

    auto& root = m_device->get_root_node();

    strip = ossia::create_parameter(root, "/leds", "list");
    for (int i = 0; i < m_pixels; i++)
    {
      pixels.push_back(ossia::create_parameter(
          strip->get_node(), std::to_string(i), "rgb"));
    }
    if (m_fd >= 0)
    {
      switch (m_format)
      {
        case NeoPixelsFormat::GRB:
          m_timer.start(
              [this]
              {
                init_bitbang();
                update_function_grb();
                push_to_spi();
              });
          break;
        case NeoPixelsFormat::RGB:
          m_timer.start(
              [this]
              {
                init_bitbang();
                update_function_rgb();
                push_to_spi();
              });
          break;
      }
    }
  }

  static auto rgb_to_bitbang(uint8_t r, uint8_t g, uint8_t b) noexcept
  {
    using namespace std;
    const auto r_array = int_to_neopixel(r);
    const auto g_array = int_to_neopixel(g);
    const auto b_array = int_to_neopixel(b);

    std::array<uint8_t, 24> fullColor;
#pragma omp simd
    for (int cnt = 0; cnt < 8; ++cnt)
    {
      fullColor[cnt + 0] = r_array[cnt];
      fullColor[cnt + 8] = g_array[cnt];
      fullColor[cnt + 16] = b_array[cnt];
    }
    return fullColor;
  }

  bool pull(ossia::net::parameter_base& v) override { return false; }

  bool
  push(const ossia::net::parameter_base& p, const ossia::value& v) override
  {
    return false;
  }

  bool push_raw(const ossia::net::full_parameter_data&) override
  {
    return false;
  }

  bool observe(ossia::net::parameter_base&, bool) override { return false; }

  bool update(ossia::net::node_base& node_base) override { return false; }

  void init_bitbang()
  {
    m_bitbang_data.clear();
    m_bitbang_data.reserve(m_pixels * 24);
  }

  void push_to_spi()
  {
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = reinterpret_cast<std::uintptr_t>(m_bitbang_data.data());
    spi.len = m_pixels * 24;
    spi.delay_usecs = 0;
    spi.speed_hz = m_speed * 8 * 1024;
    spi.bits_per_word = 8;
    ioctl(m_fd, SPI_IOC_MESSAGE(1), &spi);
  }

  void update_function_rgb()
  {
    const int N = std::min(m_pixels, (int)std::ssize(pixels));
#pragma omp simd
    for (int i = 0; i < N; i++)
    {
      auto col = ossia::convert<ossia::vec3f>(pixels[i]->value());

      // 1. ossia::vec3f to basic rgb
      const uint8_t r = std::clamp(col[0], 0.f, 1.f) * 255;
      const uint8_t g = std::clamp(col[1], 0.f, 1.f) * 255;
      const uint8_t b = std::clamp(col[2], 0.f, 1.f) * 255;

      // 2. rgb to bitbangable rgb
      const auto& r_array = int_to_neopixel_table[r];
      const auto& g_array = int_to_neopixel_table[g];
      const auto& b_array = int_to_neopixel_table[b];

      uint8_t* data = m_bitbang_data.data() + i * 24;
      std::copy_n(r_array.data(), 8, data);
      std::copy_n(g_array.data(), 8, data + 8);
      std::copy_n(b_array.data(), 8, data + 16);
    }
  }

  void update_function_grb()
  {
    const int N = std::min(m_pixels, (int)std::ssize(pixels));
#pragma omp simd
    for (int i = 0; i < N; i++)
    {
      auto col = ossia::convert<ossia::vec3f>(pixels[i]->value());

      // 1. ossia::vec3f to basic rgb
      const uint8_t g = std::clamp(col[1], 0.f, 1.f) * 255;
      const uint8_t r = std::clamp(col[0], 0.f, 1.f) * 255;
      const uint8_t b = std::clamp(col[2], 0.f, 1.f) * 255;

      // 2. rgb to bitbangable grb
      const auto& g_array = int_to_neopixel_table[g];
      const auto& r_array = int_to_neopixel_table[r];
      const auto& b_array = int_to_neopixel_table[b];

      uint8_t* data = m_bitbang_data.data() + i * 24;
      std::copy_n(g_array.data(), 8, data);
      std::copy_n(r_array.data(), 8, data + 8);
      std::copy_n(b_array.data(), 8, data + 16);
    }
  }

  ossia::net::network_context_ptr m_context;
  ossia::net::device_base* m_device{};
  ossia::timer m_timer;

  int m_pixels{};
  int m_speed{};
  NeoPixelsFormat m_format{};

  std::vector<uint8_t> m_bitbang_data;
  ossia::net::parameter_base* strip{};
  std::vector<ossia::net::parameter_base*> pixels;
  int m_fd{-1};
};

DeviceImplementation::DeviceImplementation(
    const Device::DeviceSettings& settings,
    const Explorer::DeviceDocumentPlugin& plugin,
    const score::DocumentContext&)
    : OwningDeviceInterface{settings}
    , m_ctx{plugin}
{
  m_capas.canRefreshTree = true;
  m_capas.canAddNode = false;
  m_capas.canRemoveNode = false;
  m_capas.canRenameNode = false;
  m_capas.canSetProperties = false;
  m_capas.canSerialize = false;
}

DeviceImplementation::~DeviceImplementation() { }

bool DeviceImplementation::reconnect()
{
  disconnect();

  try
  {
    const auto& set
        = m_settings.deviceSpecificSettings.value<SpecificSettings>();

    // Needed by most protocols:
    auto& ctx = m_ctx.networkContext();

    auto protocol = std::make_unique<led_protocol>(set, ctx);
    auto dev = std::make_unique<ossia::net::generic_device>(
        std::move(protocol), settings().name.toStdString());

    m_dev = std::move(dev);
    deviceChanged(nullptr, m_dev.get());
  }
  catch (const std::runtime_error& e)
  {
    qDebug() << "Led error: " << e.what();
  }
  catch (...)
  {
    qDebug() << "Led error";
  }

  return connected();
}

void DeviceImplementation::disconnect()
{
  OwningDeviceInterface::disconnect();
}
}
