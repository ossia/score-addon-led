#include "LEDOutputDevice.hpp"

#include <Gfx/GfxApplicationPlugin.hpp>
#include <Gfx/GfxExecContext.hpp>
#include <Gfx/GfxParameter.hpp>
#include <Gfx/Graph/NodeRenderer.hpp>
#include <Gfx/Graph/OutputNode.hpp>
#include <Gfx/Graph/RenderList.hpp>
#include <Gfx/InvertYRenderer.hpp>

#include <score/gfx/OpenGL.hpp>
#include <score/gfx/QRhiGles2.hpp>

#include <ossia/detail/fmt.hpp>
#include <ossia/network/base/device.hpp>
#include <ossia/network/base/protocol.hpp>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/multicast.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/ip/v6_only.hpp>

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QOffscreenSurface>
#include <QSpinBox>

#include <wobjectimpl.h>

#include <span>

namespace Gfx
{
class LedOutputDevice final : public GfxOutputDevice
{
  W_OBJECT(LedOutputDevice)
public:
  using GfxOutputDevice::GfxOutputDevice;
  ~LedOutputDevice();

private:
  bool reconnect() override;
  ossia::net::device_base* getDevice() const override { return m_dev.get(); }

  gfx_protocol_base* m_protocol{};
  mutable std::unique_ptr<ossia::net::device_base> m_dev;
};

class LedOutputSettingsWidget final : public Gfx::SharedOutputSettingsWidget
{
public:
  LedOutputSettingsWidget(QWidget* parent = nullptr);

  Device::DeviceSettings getSettings() const override;
};

}
W_OBJECT_IMPL(Gfx::LedOutputDevice)

namespace Gfx
{
struct LedOutputNode final : score::gfx::OutputNode
{
  score::gfx::OutputNode& m_parent;
  std::weak_ptr<score::gfx::RenderList> m_renderer{};
  QRhiTexture* m_texture{};
  QRhiTextureRenderTarget* m_renderTarget{};
  std::function<void()> m_update;
  std::shared_ptr<score::gfx::RenderState> m_renderState{};

  explicit LedOutputNode(OutputNode& parent, const SharedOutputSettings& set)
      : OutputNode{}
      , m_parent{parent}
      , m_settings{set}
  {
    input.push_back(new score::gfx::Port{this, {}, score::gfx::Types::Image, {}});
  }

  ~LedOutputNode() { }
  bool canRender() const override { return true; }

  void startRendering() override { }

  void render() override
  {
    if(m_update)
      m_update();

    auto renderer = m_renderer.lock();
    if(renderer && m_renderState)
    {
      auto rhi = m_renderState->rhi;
      QRhiCommandBuffer* cb{};
      if(rhi->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess)
        return;

      renderer->render(*cb);
      rhi->endOffscreenFrame();
    }
  }

  const QRhiReadbackResult& readPixels() noexcept
  {
    this->render();
    return m_readback;
  }

  score::gfx::OutputNode::Configuration configuration() const noexcept override
  {
    return {.manualRenderingRate = {}, .outputNeedsRenderPass = {}, .parent = &m_parent};
  }

  void onRendererChange() override { }

  void stopRendering() override { }

  void setRenderer(std::shared_ptr<score::gfx::RenderList> r) override
  {
    m_renderer = r;
  }

  score::gfx::RenderList* renderer() const override { return m_renderer.lock().get(); }

  void createOutput(
      score::gfx::GraphicsApi graphicsApi, std::function<void()> onReady,
      std::function<void()> onUpdate, std::function<void()> onResize) override
  {
    m_renderState = std::make_shared<score::gfx::RenderState>();
    m_update = onUpdate;

    m_renderState->surface = QRhiGles2InitParams::newFallbackSurface();
    QRhiGles2InitParams params;
    params.fallbackSurface = m_renderState->surface;
    score::GLCapabilities caps;
    caps.setupFormat(params.format);
    m_renderState->rhi = QRhi::create(QRhi::OpenGLES2, &params, {});
    m_renderState->renderSize = QSize(m_settings.width, m_settings.height);
    m_renderState->outputSize = m_renderState->renderSize;
    m_renderState->api = score::gfx::GraphicsApi::OpenGL;
    m_renderState->version = caps.qShaderVersion;

    auto rhi = m_renderState->rhi;
    m_texture = rhi->newTexture(
        QRhiTexture::RGBA8, m_renderState->renderSize, 1,
        QRhiTexture::RenderTarget | QRhiTexture::UsedAsTransferSource);
    m_texture->create();
    m_renderTarget = rhi->newTextureRenderTarget({m_texture});
    m_renderState->renderPassDescriptor
        = m_renderTarget->newCompatibleRenderPassDescriptor();
    m_renderTarget->setRenderPassDescriptor(m_renderState->renderPassDescriptor);
    m_renderTarget->create();

    onReady();
  }

  void destroyOutput() override { }

  std::shared_ptr<score::gfx::RenderState> renderState() const override
  {
    return m_renderState;
  }

  score::gfx::OutputNodeRenderer*
  createRenderer(score::gfx::RenderList& r) const noexcept override
  {
    score::gfx::TextureRenderTarget rt{
        m_texture, nullptr, nullptr, m_renderState->renderPassDescriptor,
        m_renderTarget};
    return new Gfx::InvertYRenderer{
        *this, rt, const_cast<QRhiReadbackResult&>(m_readback)};
  }

  SharedOutputSettings m_settings;

  QRhiReadbackResult m_readback;
};

namespace
{
#pragma pack(push, 1)
struct e131_acn_root_layer
{                          /* ACN Root Layer: 38 bytes */
  uint16_t preamble_size;  /* Preamble Size */
  uint16_t postamble_size; /* Post-amble Size */
  uint8_t acn_pid[12];     /* ACN Packet Identifier */
  uint16_t flength;        /* Flags (high 4 bits) & Length (low 12 bits) */
  uint32_t vector;         /* Layer Vector */
  uint8_t cid[16];         /* Component Identifier (UUID) */
};

struct e131_framing
{
  uint16_t flength;        /* Flags (high 4 bits) & Length (low 12 bits) */
  uint32_t vector;         /* Layer Vector */
  uint8_t source_name[64]; /* User Assigned Name of Source (UTF-8) */
  uint8_t priority;        /* Packet Priority (0-200, default 100) */
  uint16_t reserved;       /* Reserved (should be always 0) */
  uint8_t seq_number; /* Sequence Number (detect duplicates or out of order
                              packets) */
  uint8_t options;    /* Options Flags (bit 7: preview data, bit 6: stream
                              terminated) */
  uint16_t universe;  /* DMX Universe Number */
};

struct e131_device_management_protocol
{                      /* Device Management Protocol (DMP) Layer: 523 bytes */
  uint16_t flength;    /* Flags (high 4 bits) / Length (low 12 bits) */
  uint8_t vector;      /* Layer Vector */
  uint8_t type;        /* Address Type & Data Type */
  uint16_t first_addr; /* First Property Address */
  uint16_t addr_inc;   /* Address Increment */
  uint16_t prop_val_cnt; /* Property Value Count (1 + number of slots) */
  uint8_t prop_val[513]; /* Property Values (DMX start code + slots data) */
};

struct e131_packet
{
  e131_acn_root_layer root;
  e131_framing frame;
  e131_device_management_protocol dmp;
};

/* E1.31 Framing Options Type */
enum class e131_option_t
{
  E131_OPT_TERMINATED = 6,
  E131_OPT_PREVIEW = 7,
};

/* Initialize an E1.31 packet using a universe and a number of slots */
static int e131_pkt_init(
    e131_packet* packet,
    const uint16_t universe,
    const uint16_t num_slots)
{
  if (packet == NULL || universe < 1 || universe > 63999 || num_slots < 1
      || num_slots > 512)
  {
    errno = EINVAL;
    return -1;
  }

  // compute packet layer lengths
  const uint16_t prop_val_cnt = num_slots + 1;
  const uint16_t dmp_length
      = prop_val_cnt + sizeof packet->dmp - sizeof packet->dmp.prop_val;
  const uint16_t frame_length = sizeof packet->frame + dmp_length;
  const uint16_t root_length = sizeof packet->root.flength
                               + sizeof packet->root.vector
                               + sizeof packet->root.cid + frame_length;

  // clear packet
  memset(packet, 0, sizeof *packet);

  /* E1.31 Private Constants */
  static constexpr uint16_t _E131_PREAMBLE_SIZE = 0x0010;
  static constexpr uint16_t _E131_POSTAMBLE_SIZE = 0x0000;
  static constexpr uint8_t _E131_ACN_PID[] = {
      0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00};
  static constexpr uint32_t _E131_ROOT_VECTOR = 0x00000004;
  static constexpr uint32_t _E131_FRAME_VECTOR = 0x00000002;
  static constexpr uint8_t _E131_DMP_VECTOR = 0x02;
  static constexpr uint8_t _E131_DMP_TYPE = 0xa1;
  static constexpr uint16_t _E131_DMP_FIRST_ADDR = 0x0000;
  static constexpr uint16_t _E131_DMP_ADDR_INC = 0x0001;

  // set Root Layer values
  packet->root.preamble_size = htons(_E131_PREAMBLE_SIZE);
  packet->root.postamble_size = htons(_E131_POSTAMBLE_SIZE);
  memcpy(packet->root.acn_pid, _E131_ACN_PID, sizeof packet->root.acn_pid);
  packet->root.flength = htons(0x7000 | root_length);
  packet->root.vector = htonl(_E131_ROOT_VECTOR);

  // char uuid[17] = {
  // "\xfb\x3c\x10\x65\xa1\x7f\x4d\xe2\x99\x19\x31\x7a\x07\xc1\x00\x52" };
  // memcpy(packet->root.cid, uuid, 16);

  // set Framing Layer values
  packet->frame.flength = htons(0x7000 | frame_length);
  packet->frame.vector = htonl(_E131_FRAME_VECTOR);
  memcpy(packet->frame.source_name, "libossia", 8);
  packet->frame.priority = 0x64;
  packet->frame.universe = htons(universe);

  // set Device Management Protocol (DMP) Layer values
  packet->dmp.flength = htons(0x7000 | dmp_length);
  packet->dmp.vector = _E131_DMP_VECTOR;
  packet->dmp.type = _E131_DMP_TYPE;
  packet->dmp.first_addr = htons(_E131_DMP_FIRST_ADDR);
  packet->dmp.addr_inc = htons(_E131_DMP_ADDR_INC);
  packet->dmp.prop_val_cnt = htons(prop_val_cnt);

  return 0;
}

#pragma pack(pop)
static_assert(sizeof(e131_packet) == 638);
}

struct E131Config
{
  static constexpr uint16_t default_port = 5568;
  static constexpr uint8_t default_priority = 100;
  std::string host;
  int port{default_port};
  int channels_per_universe{512};
  bool multicast{};
  enum send_mode
  {
    always,
    only_updated
  };
};
struct LEDConfigDMX
{
  int start_universe{};
  int start_address{};
  int pixels{};
  int pixel_stride{};
  enum
  {
    Zebra
  } stripe_mode{};
  enum
  {
    XPos,
    XNeg,
  } hdir{};
  enum
  {
    YUp,
    YDown,
  } vdir{};
};
struct LEDDMX
{
  LEDConfigDMX config;
  std::vector<uint8_t> arr;
};

struct LEDSenderDMX
{
  explicit LEDSenderDMX(std::span<LEDConfigDMX> conf)
  {
    led_config.reserve(conf.size());
    for (const auto& stripe : conf)
    {
      led_config.push_back(LEDDMX{.config = stripe, .arr = {}});
      led_config.back().arr.resize(stripe.pixels * 3);
    }
  }

  // 1. We write for each LED their data
  void set(int led, std::span<const uint8_t> rgba)
  {
    SCORE_ASSERT(led >= 0);
    SCORE_ASSERT(std::ssize(led_config) > led);
    SCORE_ASSERT(!rgba.empty());
    SCORE_ASSERT(rgba.size() % 4 == 0);
    auto& stripe = led_config[led];
    SCORE_ASSERT(std::ssize(rgba) == (stripe.config.pixels * 4));

    const int rgba_N = rgba.size();
    int rgb_i = 0;
    int rgba_i = 0;
#pragma omp simd
    for (rgba_i = 0; rgba_i < rgba_N; rgba_i += 4)
    {
      stripe.arr[rgb_i + 0] = rgba[rgba_i + 0];
      stripe.arr[rgb_i + 1] = rgba[rgba_i + 1];
      stripe.arr[rgb_i + 2] = rgba[rgba_i + 2];
      rgb_i += 3;
    }
  }

  std::vector<LEDDMX> led_config;
};

struct LEDSenderE131Unicast : LEDSenderDMX
{
  E131Config configuration;
  boost::asio::io_context ctx;
  boost::asio::ip::udp::socket sock;
  boost::asio::ip::udp::endpoint ep;

  explicit LEDSenderE131Unicast(
      const E131Config& conf,
      std::span<LEDConfigDMX> leds)
      : LEDSenderDMX{leds}
      , configuration{conf}
      , sock{ctx}
      , ep{boost::asio::ip::make_address(conf.host), (uint16_t)conf.port}
  {

    // 1. Check how many DMX universes we're going to need
    for (auto& stripe : this->led_config)
    {
      int u = stripe.config.start_universe;
      // ex. RGB without spacing: stride == 3
      int channels = stripe.config.start_address
                     + stripe.config.pixels * stripe.config.pixel_stride;
      while (channels > conf.channels_per_universe)
      {
        u++;
        channels -= conf.channels_per_universe;
      }

      for (int i = stripe.config.start_universe; i <= u; i++)
        universes.emplace(i, nullptr);
    }

    // 2. Pre-create the packet headers
    packets.resize(universes.size());
    {
      int i = 0;
      for (auto& [uni, ptr] : universes)
      {
        auto& packet = packets[i];
        ptr = &packet;

        // e1.31 is 1-based
        e131_pkt_init(&packet, uni + 1, conf.channels_per_universe);
        // FIXME set a unique CID
        i++;
      }
    }

    // 2. Open the socket
    boost::system::error_code ec;
    sock.open(boost::asio::ip::udp::v6(), ec);
    sock.set_option(boost::asio::ip::v6_only{false}, ec);
    sock.set_option(boost::asio::ip::udp::socket::reuse_address(true), ec);
    sock.set_option(boost::asio::socket_base::broadcast(true), ec);
  }

  void push()
  {
    // 1. Copy all the LED data in their universe
    for (auto& stripe : led_config)
    {
      int u = stripe.config.start_universe;
      int a = stripe.config.start_address;
      while (a > this->configuration.channels_per_universe)
      {
        a -= this->configuration.channels_per_universe;
        u++;
      }

      SCORE_ASSERT(universes.contains(u));
      SCORE_ASSERT(universes[u]);
      auto* current_pkt = universes[u];

      auto* bytes = stripe.arr.data();
      auto* end = bytes + stripe.arr.size();
      while (bytes != end)
      {
        if (a < this->configuration.channels_per_universe)
        {
          [[likely]];
          current_pkt->dmp.prop_val[1 + a] = *bytes;
          bytes++;
          a++;
        }
        else
        {
          [[unlikely]];
          a = 0;
          u++;
        }
      }
    }

    boost::system::error_code ec;
    for (auto& packet : packets)
    {
      packet.frame.seq_number++; // = seqnum;
      sock.send_to(
          boost::asio::const_buffer(&packet, sizeof(packet)), ep, 0, ec);
      //seqnum++;
    }
  }

  ossia::flat_map<int, e131_packet*> universes;
  std::vector<e131_packet> packets;
  uint8_t seqnum{};
};

struct LEDSenderE131Multicast : LEDSenderDMX
{
  boost::asio::io_context ctx;
  std::vector<boost::asio::ip::udp::socket> sock;

  explicit LEDSenderE131Multicast(
      const E131Config& conf,
      std::span<LEDConfigDMX> leds)
      : LEDSenderDMX{std::move(leds)}
  {
  }

  void open_socket(
      const E131Config& conf,
      int universe,
      boost::asio::ip::udp::socket& sock)
  {
    boost::system::error_code ec;
    sock.open(boost::asio::ip::udp::v6(), ec);
    sock.set_option(boost::asio::ip::v6_only{false}, ec);
    sock.set_option(boost::asio::ip::udp::socket::reuse_address(true), ec);
    sock.set_option(boost::asio::socket_base::broadcast(true), ec);

    if (!conf.host.empty())
    {
      auto outbound_address = boost::asio::ip::make_address(conf.host).to_v4();
      auto mcast_address = boost::asio::ip::address_v4(0xefff0000 | universe);

      sock.set_option(boost::asio::ip::multicast::enable_loopback(false));
      sock.set_option(
          boost::asio::ip::multicast::outbound_interface(outbound_address));
      sock.set_option(boost::asio::ip::multicast::join_group(
          mcast_address, outbound_address));
    }
  }
  void push()
  {
    // boost::system::error_code ec;
    // m_socket.send_to(boost::asio::const_buffer(data, sz), ep, 0, ec);
  }
};

struct LedParentOutputNode : score::gfx::OutputNode
{
  std::function<void()> m_update;
  ossia::net::node_base& m_root;
  LEDSenderE131Unicast* sender{};

  explicit LedParentOutputNode(
      ossia::net::node_base& root, const SharedOutputSettings& set)
      : OutputNode{}
      , m_root{root}
      , m_settings{set}
  {
    E131Config conf;
    conf.host = "127.0.0.1";
    conf.port = conf.default_port;
    conf.channels_per_universe = 510;

    std::vector<LEDConfigDMX> stripes;
    for (int i = 0; i < 8; i++)
      stripes.push_back(LEDConfigDMX{
          .start_universe = 0,
          .start_address = i * 32 * 3,
          .pixels = 32,
          .pixel_stride = 3});
    sender = new LEDSenderE131Unicast(conf, stripes);
  }

  ~LedParentOutputNode() { }
  bool canRender() const override { return true; }

  void startRendering() override { }

  void render() override
  {
    if(m_update)
      m_update();

    int stripe_i = 0;
    for(auto& cld : m_root.children_copy())
    {
      auto& gfx = (gfx_parameter_base&)*cld->get_parameter();
      auto n = dynamic_cast<LedOutputNode*>(gfx.node);
      const auto& rb = n->readPixels();

      int sz = rb.pixelSize.width() * rb.pixelSize.height() * 4;
      int bytes = rb.data.size();
      {
        if (bytes > 0 && bytes >= sz)
        {
          // rb.data is RGBA, we go to RGB

          sender->set(
              stripe_i++,
              std::span<const uint8_t>(
                  (const uint8_t*)rb.data.constData(), rb.data.size()));
        }
      }
    }
    sender->push();
  }

  score::gfx::OutputNode::Configuration configuration() const noexcept override
  {
    return {.manualRenderingRate = 1000. / m_settings.rate};
  }

  void onRendererChange() override { }

  void stopRendering() override { }

  void setRenderer(std::shared_ptr<score::gfx::RenderList> r) override { }

  score::gfx::RenderList* renderer() const override { return nullptr; }

  void createOutput(
      score::gfx::GraphicsApi graphicsApi, std::function<void()> onReady,
      std::function<void()> onUpdate, std::function<void()> onResize) override
  {
    m_renderState = std::make_shared<score::gfx::RenderState>();
    onReady();
  }

  void destroyOutput() override { }

  std::shared_ptr<score::gfx::RenderState> renderState() const override
  {
    return m_renderState;
  }

  score::gfx::OutputNodeRenderer*
  createRenderer(score::gfx::RenderList& r) const noexcept override
  {
    class DummyRenderer final : public score::gfx::OutputNodeRenderer
    {
    public:
      using score::gfx::OutputNodeRenderer::OutputNodeRenderer;

      score::gfx::TextureRenderTarget
      renderTargetForInput(const score::gfx::Port& p) override
      {
        return {};
      }

      void finishFrame(
          score::gfx::RenderList& renderer, QRhiCommandBuffer& cb,
          QRhiResourceUpdateBatch*& res) override
      {
      }

      void init(score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res) override
      {
      }
      void update(
          score::gfx::RenderList& renderer, QRhiResourceUpdateBatch& res,
          score::gfx::Edge* edge) override
      {
      }
      void release(score::gfx::RenderList&) override { }

      void updateReadback(QRhiReadbackResult& rb) { }
    };
    return new DummyRenderer{*this};
  }

  SharedOutputSettings m_settings;
  std::shared_ptr<score::gfx::RenderState> m_renderState;
};

}

namespace Gfx
{

class led_output_device : public ossia::net::device_base
{
  gfx_node_base root;

public:
  led_output_device(
      const SharedOutputSettings& set, std::unique_ptr<gfx_protocol_base> proto,
      std::string name)
      : ossia::net::device_base{std::move(proto)}
      , root{
            *this, *static_cast<gfx_protocol_base*>(m_protocol.get()),
            new LedParentOutputNode{root, set}, name}
  {
    auto& parent_node = (score::gfx::OutputNode&)*root.get_parameter()->node;
    for (int i = 0; i < 8; i++)
    {
      root.add_child(std::make_unique<gfx_node_base>(
          *this, *static_cast<gfx_protocol_base*>(m_protocol.get()),
          new LedOutputNode{parent_node, set}, "strip." + std::to_string(i + 1)));
    }
  }

  const gfx_node_base& get_root_node() const override { return root; }
  gfx_node_base& get_root_node() override { return root; }
};

LedOutputDevice::~LedOutputDevice() { }

bool LedOutputDevice::reconnect()
{
  disconnect();

  try
  {
    auto plug = m_ctx.findPlugin<DocumentPlugin>();
    if(plug)
    {
      auto set = m_settings.deviceSpecificSettings.value<SharedOutputSettings>();
      m_protocol = new gfx_protocol_base{plug->exec};
      m_dev = std::make_unique<led_output_device>(
          set, std::unique_ptr<gfx_protocol_base>(m_protocol),
          m_settings.name.toStdString());
    }
  }
  catch(std::exception& e)
  {
    qDebug() << "Could not connect: " << e.what();
  }
  catch(...)
  {
    // TODO save the reason of the non-connection.
  }

  return connected();
}

Device::ProtocolSettingsWidget* LedOutputProtocolFactory::makeSettingsWidget()
{
  return new LedOutputSettingsWidget{};
}

QString LedOutputProtocolFactory::prettyName() const noexcept
{
  return QObject::tr("LED Output");
}

QUrl LedOutputProtocolFactory::manual() const noexcept
{
  return QUrl("https://ossia.io/score-docs/devices/led-device.html");
}

QString LedOutputProtocolFactory::category() const noexcept
{
  return StandardCategories::lights;
}
Device::DeviceInterface* LedOutputProtocolFactory::makeDevice(
    const Device::DeviceSettings& settings, const Explorer::DeviceDocumentPlugin& doc,
    const score::DocumentContext& ctx)
{
  return new LedOutputDevice(settings, ctx);
}

const Device::DeviceSettings& LedOutputProtocolFactory::defaultSettings() const noexcept
{
  static const Device::DeviceSettings settings = [&]() {
    Device::DeviceSettings s;
    s.protocol = concreteKey();
    s.name = "Led Output";
    SharedOutputSettings set;
    set.width = 32;
    set.height = 1;
    set.path = "/tmp/score_shm_video";
    set.rate = 44.;
    s.deviceSpecificSettings = QVariant::fromValue(set);
    return s;
  }();
  return settings;
}

LedOutputSettingsWidget::LedOutputSettingsWidget(QWidget* parent)
    : SharedOutputSettingsWidget{parent}
{
  m_deviceNameEdit->setText("Led Out");
  ((QLabel*)m_layout->labelForField(m_shmPath))->setText("Led path");

  auto helpLabel
      = new QLabel{tr("To test, use the following command: \n"
                      "$ gst-launch-1.0 ledsrc socket-path=<THE PATH> ! "
                      "videoconvert ! xvimagesink")};
  helpLabel->setTextInteractionFlags(Qt::TextInteractionFlag::TextSelectableByMouse);
  m_layout->addRow(helpLabel);

  setSettings(LedOutputProtocolFactory{}.defaultSettings());
}

Device::DeviceSettings LedOutputSettingsWidget::getSettings() const
{
  auto set = SharedOutputSettingsWidget::getSettings();
  set.protocol = LedOutputProtocolFactory::static_concreteKey();
  return set;
}

}
