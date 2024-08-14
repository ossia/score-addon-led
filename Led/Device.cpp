#include "Device.hpp"

#include "SpecificSettings.hpp"

#include <Explorer/DocumentPlugin/DeviceDocumentPlugin.hpp>

#include <score/document/DocumentContext.hpp>

#include <ossia/detail/timer.hpp>
#include <ossia/detail/triple_buffer.hpp>
#include <ossia/network/base/device.hpp>
#include <ossia/network/base/node.hpp>
#include <ossia/network/base/parameter.hpp>
#include <ossia/network/base/protocol.hpp>
#include <ossia/network/common/complex_type.hpp>
#include <ossia/network/common/device_parameter_t.hpp>
#include <ossia/network/context.hpp>
#include <ossia/network/generic/generic_device.hpp>

#include <boost/container/static_vector.hpp>

#include <QDebug>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <wobjectimpl.h>

#include <iostream>
#include <span>

W_OBJECT_IMPL(Led::DeviceImplementation)

namespace Led
{
class pixel_vec_parameter : public ossia::net::device_parameter_t<std::string>
{
public:
  pixel_vec_parameter(ossia::net::node_base& node, std::span<uint8_t> buffer)
      : device_parameter_t(node)
      , m_buffer{buffer}
  {
  }

  ~pixel_vec_parameter() = default;

  ossia::value set_value(const ossia::value& val) override
  {
    if (val.valid())
    {
      if (auto str = val.target<std::string>())
      {
        m_current_value = *str;
      }
      else if (auto vec = val.target<std::vector<ossia::value>>())
      {
        m_current_value.clear();
        m_current_value.reserve(vec->size());
        for (auto& val : *vec)
        {
          m_current_value.push_back(ossia::convert<int>(val));
        }
      }
      else if (auto vec = val.target<ossia::vec3f>())
      {
        m_current_value.clear();
        m_current_value.resize(3);
        m_current_value[0] = (*vec)[0] * 255;
        m_current_value[1] = (*vec)[1] * 255;
        m_current_value[2] = (*vec)[2] * 255;
      }
      send(val);
      device_update_value();
    }

    return m_current_value;
  }

private:
  void device_update_value() override
  {
    auto& col = m_current_value;
    int N = std::min(col.size(), m_buffer.size());
    std::copy_n(col.begin(), N, m_buffer.begin());
  }

  std::span<uint8_t> m_buffer;
};
class pixel_fill_parameter
    : public ossia::net::device_parameter_t<ossia::vec3f>
{
public:
  pixel_fill_parameter(ossia::net::node_base& node, std::span<uint8_t> buffer)
      : device_parameter_t(node)
      , m_buffer{buffer}
  {
  }

  ~pixel_fill_parameter() = default;

private:
  void device_update_value() override
  {
    auto& col = m_current_value;

    const uint8_t r = std::clamp(col[0], 0.f, 1.f) * 255.0f;
    const uint8_t g = std::clamp(col[1], 0.f, 1.f) * 255.0f;
    const uint8_t b = std::clamp(col[2], 0.f, 1.f) * 255.0f;

    auto* const pix = m_buffer.data();
    const int N = m_buffer.size() / 3;
#pragma omp simd
    for (int i = 0; i < N; i++)
    {
      pix[i * 3 + 0] = r;
      pix[i * 3 + 1] = g;
      pix[i * 3 + 2] = b;
    }
  }

  std::span<uint8_t> m_buffer;
};
class pixel_parameter : public ossia::net::device_parameter_t<ossia::vec3f>
{
public:
  pixel_parameter(ossia::net::node_base& node, std::span<uint8_t, 3> buffer)
      : device_parameter_t(node)
      , m_buffer{buffer}
  {
  }

  ~pixel_parameter() = default;

private:
  void device_update_value() override
  {
    auto& col = m_current_value;
    {
      const uint8_t r = std::clamp(col[0], 0.f, 1.f) * 255.0f;
      const uint8_t g = std::clamp(col[1], 0.f, 1.f) * 255.0f;
      const uint8_t b = std::clamp(col[2], 0.f, 1.f) * 255.0f;

      auto* const pixel = m_buffer.data();
      pixel[0] = r;
      pixel[1] = g;
      pixel[2] = b;
    }
  }

  std::span<uint8_t, 3> m_buffer;
};

// https://www.hackster.io/RVLAD/neopixel-ws2812b-spi-driver-with-ada-on-stm32f4-discovery-d330ea
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

static constexpr std::array<std::array<uint8_t, 8>, 256> int_to_neopixel_table
    = []() constexpr
{
  std::array<std::array<uint8_t, 8>, 256> res;
  for (int i = 0; i < 256; i++)
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
    m_rgb_data.resize(m_pixels * 3);
    using namespace std::literals;
    m_fd = ::open(set.device.toStdString().c_str(), O_RDWR);
    if (m_fd < 0)
    {
      qDebug() << "Could not open " + set.device;
      return;
    }

    auto fps = std::clamp(set.fps, 0.001f, 1000.f);
    if (set.fps <= 0.f)
      fps = 30.;
    m_timer.set_delay(
        std::chrono::milliseconds{static_cast<int>(1000.0f / fps)});
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

    strip = root.create_child("leds");
    strip->set_parameter(
        std::make_unique<pixel_vec_parameter>(*strip, m_rgb_data));

    auto fill = strip->create_child("fill");
    fill->set_parameter(
        std::make_unique<pixel_fill_parameter>(*fill, m_rgb_data));
    for (int i = 0; i < m_pixels; i++)
    {
      auto node = strip->create_child(std::to_string(i));
      node->set_parameter(std::make_unique<pixel_parameter>(
          *node, std::span<uint8_t, 3>(m_rgb_data.data() + i * 3, 3)));
    }

    if (m_fd >= 0)
    {
      m_bitbang_data.resize(m_pixels * 24, boost::container::default_init);
      switch (m_format)
      {
        case NeoPixelsFormat::GRB:
          m_timer.start(
              [this]
              {
                update_function_grb();
                push_to_spi();
              });
          break;
        case NeoPixelsFormat::RGB:
          m_timer.start(
              [this]
              {
                update_function_rgb();
                push_to_spi();
              });
          break;
      }
    }
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
    const int N = m_pixels;

#pragma omp simd
    for (int i = 0; i < N; i++)
    {
      const auto* const src = m_rgb_data.data() + i * 3;
      uint8_t r = src[0];
      uint8_t g = src[1];
      uint8_t b = src[2];

      auto* const dst = m_bitbang_data.data() + i * 24;
      std::copy_n(int_to_neopixel_table[r].data(), 8, dst);
      std::copy_n(int_to_neopixel_table[g].data(), 8, dst + 8);
      std::copy_n(int_to_neopixel_table[b].data(), 8, dst + 16);
    }
  }

  void update_function_grb()
  {
    const int N = m_pixels;

#pragma omp simd
    for (int i = 0; i < N; i++)
    {
      const auto* const src = m_rgb_data.data() + i * 3;
      uint8_t r = src[0];
      uint8_t g = src[1];
      uint8_t b = src[2];

      auto* const dst = m_bitbang_data.data() + i * 24;
      std::copy_n(int_to_neopixel_table[g].data(), 8, dst);
      std::copy_n(int_to_neopixel_table[r].data(), 8, dst + 8);
      std::copy_n(int_to_neopixel_table[b].data(), 8, dst + 16);
    }
  }

  ossia::net::network_context_ptr m_context;
  ossia::net::device_base* m_device{};
  ossia::timer m_timer;

  int m_pixels{};
  int m_speed{};
  NeoPixelsFormat m_format{};

  ossia::pod_vector<uint8_t> m_rgb_data;
  ossia::pod_vector<uint8_t> m_bitbang_data;
  ossia::net::node_base* strip{};
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
