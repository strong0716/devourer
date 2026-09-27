#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "AdapterCaps.h"
#include "ChannelFreq.h"
#include "IRadio.h"
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "SignalStop.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"
#include "OpenHdSingleTime.h"

namespace {

// Independent Devourer-side implementation of the byte contract documented
// by OpenHD's RadioIpcProtocol.h. The GPL-2.0 service neither includes nor
// links OpenHD source.
constexpr std::uint32_t kMagic = 0x4f484452;  // "OHDR"
constexpr std::uint16_t kVersion = 2;
constexpr std::size_t kHeaderSize = 16;
constexpr std::size_t kMaxPayload = 16384;
constexpr std::size_t kMaxMessage = kHeaderSize + kMaxPayload;
constexpr std::uint16_t kHello = 1;
constexpr std::uint16_t kReady = 2;
constexpr std::uint16_t kSetFixedRf = 3;
constexpr std::uint16_t kSetTxPower = 4;
constexpr std::uint16_t kTxPacket = 5;
constexpr std::uint16_t kRxPacket = 6;
constexpr std::uint16_t kResponse = 7;
constexpr std::uint16_t kDeviceError = 8;
constexpr std::uint16_t kStop = 9;
constexpr std::uint16_t kTxError = 10;
constexpr std::uint16_t kSingleTimeControl = 11;
constexpr std::uint16_t kTimedControlBeacon = 12;
constexpr std::uint16_t kTimedDataPacket = 13;
constexpr std::uint16_t kCapabilityFixedRf = 1U << 0;
constexpr std::uint16_t kCapabilityTxPowerIndex = 1U << 1;
constexpr std::uint16_t kCapabilityFixedTdma = 1U << 2;
constexpr std::uint16_t kCapabilityFhss = 1U << 3;
constexpr std::uint16_t kCapabilityFhssTdma = 1U << 4;
// Production remains closed until each mode has real RF timing evidence.
// Lab builds may opt into individual modes without changing the air protocol.
constexpr std::uint8_t kValidatedTimedModes = 0;
constexpr std::uint8_t kTimedModes =
    kValidatedTimedModes | OPENHD_TIMED_LAB_MODES;
static_assert(kTimedModes <= 7);

std::uint8_t timed_mode_bit(std::uint8_t flags) {
  const bool fhss = (flags & openhd_single_time::kFhssFlag) != 0;
  const bool tdma = (flags & openhd_single_time::kTdmaFlag) != 0;
  return fhss ? (tdma ? 4U : 2U) : (tdma ? 1U : 0U);
}
// Values used by Devourer's SelectedChannel::ChannelOffset.
constexpr std::uint8_t kPrimaryOffsetDontCare = 0;
constexpr std::uint8_t kPrimaryOffsetLower = 1;
constexpr std::uint8_t kPrimaryOffsetUpper = 2;
constexpr std::uint8_t kRadiotapFcs = 0x10;
constexpr std::uint8_t kRadiotapBadFcs = 0x40;

std::uint64_t monotonic_us() {
  timespec stamp{};
  if (::clock_gettime(CLOCK_MONOTONIC, &stamp) != 0) return 0;
  return static_cast<std::uint64_t>(stamp.tv_sec) * 1000000ULL +
         static_cast<std::uint64_t>(stamp.tv_nsec) / 1000ULL;
}

struct Args {
  std::string socket_path = "/run/openhd-radio.sock";
  std::string trace_file;
  std::uint16_t vid = 0;
  std::uint16_t pid = 0;
  int bus = -1;
  std::string port;
};

struct Message {
  std::uint16_t type = 0;
  std::uint32_t sequence = 0;
  const std::uint8_t* payload = nullptr;
  std::size_t payload_size = 0;
};

void put_u8(std::vector<std::uint8_t>& out, std::uint8_t value) {
  out.push_back(value);
}

void put_u16(std::vector<std::uint8_t>& out, std::uint16_t value) {
  out.push_back(static_cast<std::uint8_t>(value >> 8));
  out.push_back(static_cast<std::uint8_t>(value));
}

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t value) {
  out.push_back(static_cast<std::uint8_t>(value >> 24));
  out.push_back(static_cast<std::uint8_t>(value >> 16));
  out.push_back(static_cast<std::uint8_t>(value >> 8));
  out.push_back(static_cast<std::uint8_t>(value));
}

void put_i32(std::vector<std::uint8_t>& out, std::int32_t value) {
  put_u32(out, static_cast<std::uint32_t>(value));
}

void put_u64(std::vector<std::uint8_t>& out, std::uint64_t value) {
  put_u32(out, static_cast<std::uint32_t>(value >> 32));
  put_u32(out, static_cast<std::uint32_t>(value));
}

bool get_u16(const std::uint8_t* data, std::size_t size,
             std::size_t& offset, std::uint16_t& value) {
  if (!data || offset > size || size - offset < 2) return false;
  value = static_cast<std::uint16_t>((data[offset] << 8) | data[offset + 1]);
  offset += 2;
  return true;
}

bool get_u32(const std::uint8_t* data, std::size_t size,
             std::size_t& offset, std::uint32_t& value) {
  if (!data || offset > size || size - offset < 4) return false;
  value = (static_cast<std::uint32_t>(data[offset]) << 24) |
          (static_cast<std::uint32_t>(data[offset + 1]) << 16) |
          (static_cast<std::uint32_t>(data[offset + 2]) << 8) |
          static_cast<std::uint32_t>(data[offset + 3]);
  offset += 4;
  return true;
}

bool get_u64(const std::uint8_t* data, std::size_t size,
             std::size_t& offset, std::uint64_t& value) {
  std::uint32_t high = 0, low = 0;
  if (!get_u32(data, size, offset, high) ||
      !get_u32(data, size, offset, low)) return false;
  value = (static_cast<std::uint64_t>(high) << 32) | low;
  return true;
}

bool decode_single_time_plan(const std::uint8_t* data, std::size_t size,
                             openhd_single_time::Plan& plan,
                             std::uint32_t& generation) {
  std::size_t offset = 1;  // operation byte
  if (!get_u32(data, size, offset, plan.period_us) ||
      !get_u32(data, size, offset, plan.dwell_us) ||
      !get_u32(data, size, offset, plan.pre_guard_us) ||
      !get_u32(data, size, offset, plan.post_guard_us) ||
      !get_u64(data, size, offset, plan.seed) ||
      !get_u32(data, size, offset, plan.channel_count) ||
      plan.channel_count > plan.frequencies_mhz.size())
    return false;
  for (std::uint32_t i = 0; i < plan.channel_count; ++i)
    if (!get_u32(data, size, offset, plan.frequencies_mhz[i])) return false;
  if (!get_u32(data, size, offset, plan.peer_timeout_us) ||
      offset + 6 != size)
    return false;
  plan.own_id = data[offset++];
  plan.flags = data[offset++];
  return get_u32(data, size, offset, generation) && generation != 0 &&
         openhd_single_time::valid(plan);
}

std::vector<std::uint8_t> encode(std::uint16_t type, std::uint32_t sequence,
                                 const std::uint8_t* payload,
                                 std::size_t payload_size) {
  if (payload_size > kMaxPayload || (payload_size && !payload)) return {};
  std::vector<std::uint8_t> out;
  out.reserve(kHeaderSize + payload_size);
  put_u32(out, kMagic);
  put_u16(out, kVersion);
  put_u16(out, type);
  put_u32(out, sequence);
  put_u32(out, static_cast<std::uint32_t>(payload_size));
  if (payload_size) out.insert(out.end(), payload, payload + payload_size);
  return out;
}

bool decode(const std::uint8_t* data, std::size_t size, Message& message) {
  if (!data || size < kHeaderSize || size > kMaxMessage) return false;
  std::size_t offset = 0;
  std::uint32_t magic = 0;
  std::uint16_t version = 0;
  std::uint32_t payload_size = 0;
  if (!get_u32(data, size, offset, magic) ||
      !get_u16(data, size, offset, version) ||
      !get_u16(data, size, offset, message.type) ||
      !get_u32(data, size, offset, message.sequence) ||
      !get_u32(data, size, offset, payload_size) || magic != kMagic ||
      version != kVersion || payload_size != size - kHeaderSize ||
      payload_size > kMaxPayload) {
    return false;
  }
  message.payload = data + kHeaderSize;
  message.payload_size = payload_size;
  return true;
}

bool parse_number(const char* text, unsigned long& value) {
  if (!text || !*text) return false;
  char* end = nullptr;
  errno = 0;
  value = std::strtoul(text, &end, 0);
  return errno == 0 && end != text && *end == '\0';
}

bool parse_args(int argc, char** argv, Args& args) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (i + 1 >= argc) return false;
    const char* value = argv[++i];
    if (key == "--socket") {
      args.socket_path = value;
    } else if (key == "--trace-file") {
      args.trace_file = value;
    } else if (key == "--vid") {
      unsigned long parsed = 0;
      if (!parse_number(value, parsed) || parsed > 0xffff) return false;
      args.vid = static_cast<std::uint16_t>(parsed);
    } else if (key == "--pid") {
      unsigned long parsed = 0;
      if (!parse_number(value, parsed) || parsed > 0xffff) return false;
      args.pid = static_cast<std::uint16_t>(parsed);
    } else if (key == "--bus") {
      unsigned long parsed = 0;
      if (!parse_number(value, parsed) || parsed > 255) return false;
      args.bus = static_cast<int>(parsed);
    } else if (key == "--port") {
      args.port = value;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", key.c_str());
      return false;
    }
  }
  return !args.socket_path.empty() && args.vid != 0 && args.pid != 0 &&
         args.bus >= 0 && !args.port.empty();
}

std::string usb_port_path(libusb_device* device) {
  std::array<std::uint8_t, 8> ports{};
  const int count = libusb_get_port_numbers(device, ports.data(), ports.size());
  if (count <= 0) return {};
  std::string path;
  for (int i = 0; i < count; ++i) {
    if (!path.empty()) path.push_back('.');
    path += std::to_string(ports[i]);
  }
  return path;
}

libusb_device_handle* open_device(libusb_context* context, const Args& args,
                                  const Logger_t& logger) {
  libusb_device** devices = nullptr;
  const ssize_t count = libusb_get_device_list(context, &devices);
  if (count < 0) return nullptr;
  libusb_device_handle* handle = nullptr;
  for (ssize_t i = 0; i < count && handle == nullptr; ++i) {
    libusb_device_descriptor descriptor{};
    if (libusb_get_device_descriptor(devices[i], &descriptor) != 0 ||
        descriptor.idVendor != args.vid || descriptor.idProduct != args.pid ||
        libusb_get_bus_number(devices[i]) != args.bus ||
        usb_port_path(devices[i]) != args.port) {
      continue;
    }
    if (libusb_open(devices[i], &handle) == 0) {
      logger->info("opened {:04x}:{:04x} bus={} port={}", args.vid, args.pid,
                   args.bus, args.port);
    }
  }
  libusb_free_device_list(devices, 1);
  return handle;
}

class RadioSession {
 public:
  enum class TxOutcome { Submitted, GateClosed, Fault };
  enum class TxClass { LegacyData, TimedData, TimedControl };
  explicit RadioSession(Logger_t logger) : m_logger(std::move(logger)) {}
  ~RadioSession() { close(); }

  RadioSession(const RadioSession&) = delete;
  RadioSession& operator=(const RadioSession&) = delete;

  bool open(const Args& args, std::string& error) {
    if (libusb_init(&m_context) != 0) {
      m_logger->error("libusb_init failed");
      error = "libusb_init failed";
      return false;
    }
    m_handle = open_device(m_context, args, m_logger);
    if (!m_handle) {
      m_logger->error("no USB device matched the requested VID/PID/topology");
      error = "no USB device matched the requested VID/PID/topology";
      return false;
    }
    m_interface = devourer::find_wifi_interface(m_handle);
    const int attached = libusb_kernel_driver_active(m_handle, m_interface);
    if (attached == 1) {
      m_kernel_driver_was_attached = true;
    } else if (attached < 0 && attached != LIBUSB_ERROR_NOT_SUPPORTED) {
      m_logger->error("cannot determine kernel driver state: {}", attached);
      error = "cannot determine kernel driver state: " +
              std::to_string(attached);
      return false;
    }

    const int claim = devourer::claim_interface_reset_reopen(
        m_context, m_handle, m_logger, true, m_usb_lock, {}, 10000);
    if (claim != 0 || !m_handle) {
      m_logger->error("Devourer could not claim/reset the USB interface: {}",
                      claim);
      error = "Devourer could not claim/reset the USB interface: " +
              std::to_string(claim);
      return false;
    }
    m_interface = devourer::find_wifi_interface(m_handle);
    m_interface_claimed = true;

    devourer::DeviceConfig config;
    config.rx.enable_with_tx = true;
    // Match OpenHD's rtl8812au monitor injector. Radiotap NOACK still clears
    // retry count per frame; other frames use the OpenHD driver's 32 retries
    // with rate fallback disabled.
    config.tx.retry_limit = 32;
    config.tx.retry_fallback = devourer::RetryFallback::Off;
    WiFiDriver driver(m_logger);
    m_radio = driver.CreateRadio(m_handle, m_context, m_usb_lock, config);
    if (!m_radio) {
      m_logger->error("Devourer did not recognize a supported radio");
      error = "Devourer did not recognize a supported radio";
      return false;
    }
    m_caps = m_radio->GetAdapterCaps();
    if (!m_caps.supported ||
        m_caps.generation != devourer::ChipGeneration::Jaguar1) {
      m_logger->error("prototype accepts Jaguar1 only; detected {}",
                      m_caps.chip_name ? m_caps.chip_name : "unknown");
      error = "prototype accepts Jaguar1 only; detected " +
              std::string(m_caps.chip_name ? m_caps.chip_name : "unknown");
      return false;
    }
    m_logger->info("radio ready: chip={} generation={} bus={} port={}",
                   m_caps.chip_name, devourer::generation_name(m_caps.generation),
                   args.bus, args.port);
    return true;
  }

  bool set_fixed_rf(std::uint32_t frequency_mhz, std::uint32_t width_mhz,
                    std::uint8_t primary_channel_offset,
                    std::string& error) {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    if (m_timed.configured()) {
      error = "stop the timed executor before changing the RF profile";
      return false;
    }
    if (!m_radio || frequency_mhz > 0xffff ||
        (width_mhz != 20 && width_mhz != 40)) {
      error = "prototype accepts fixed 20/40 MHz RF profiles only";
      return false;
    }
    const bool valid_offset =
        width_mhz == 20
            ? primary_channel_offset == kPrimaryOffsetDontCare
            : (primary_channel_offset == kPrimaryOffsetLower ||
               primary_channel_offset == kPrimaryOffsetUpper);
    if (!valid_offset) {
      error = "40 MHz requires a valid primary-channel offset";
      return false;
    }
    const int channel = devourer::freq_to_chan(
        static_cast<std::uint16_t>(frequency_mhz));
    if (channel <= 0 || channel > 255) {
      error = "frequency is outside the Devourer channel mapping";
      return false;
    }
    const SelectedChannel selected{
        .Channel = static_cast<std::uint8_t>(channel),
        .ChannelOffset = primary_channel_offset,
        .ChannelWidth = width_mhz == 40 ? CHANNEL_WIDTH_40 : CHANNEL_WIDTH_20};
    if (m_initialized &&
        (!m_radio->WaitTxIdle(10000) ||
         (timed_chip_supported() && !m_radio->WaitMacTxIdle(10000)))) {
      error = "radio TX must drain before changing the RF profile";
      return false;
    }
    try {
      if (!m_initialized) {
        m_radio->InitWrite(selected);
        m_initialized = true;
        m_rx_running = true;
        m_rx_thread = std::thread([this] {
          m_radio->StartRxLoop([this](const Packet& packet) {
            on_rx_packet(packet);
          });
          m_rx_running = false;
        });
      } else {
        m_radio->SetMonitorChannel(selected);
      }
    } catch (const std::exception& exception) {
      error = exception.what();
      return false;
    } catch (...) {
      error = "Devourer RF operation failed with an unknown exception";
      return false;
    }
    m_frequency_mhz = frequency_mhz;
    m_width_mhz = width_mhz;
    return true;
  }

  bool set_tx_power(std::int32_t openhd_index, std::string& error) {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    if (!m_radio || !m_caps.txpwr.supported || openhd_index < 0 ||
        openhd_index > 63) {
      error = "RTL8812AU TX power index must be in the range 0..63";
      return false;
    }
    // OpenHD uses 0 for the calibrated default; Devourer uses -1 to clear its
    // absolute flat TXAGC override.
    m_radio->SetTxPowerIndexOverride(openhd_index == 0 ? -1 : openhd_index);
    return true;
  }

  TxOutcome send_packet(const std::uint8_t* data, std::size_t length,
                        TxClass tx_class, std::uint32_t generation) {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    if (!m_radio || !m_initialized || !data || !length)
      return TxOutcome::Fault;
    if (m_timed_fault) return TxOutcome::Fault;
    if (!m_timed.configured()) {
      if (tx_class != TxClass::LegacyData || generation != 0)
        return TxOutcome::GateClosed;
      return m_radio->send_packet(data, length) ? TxOutcome::Submitted
                                                : TxOutcome::Fault;
    }
    if (tx_class == TxClass::LegacyData ||
        generation != m_timed_generation)
      return TxOutcome::GateClosed;
    const auto airtime = openhd_single_time::tx_airtime_us(data, length);
    if (!airtime) {
      m_logger->error("timed TX fault: unsupported frame length={}", length);
      m_timed_fault = true;
      return TxOutcome::Fault;
    }
    if (!m_radio->WaitTxIdle(2000)) {
      m_logger->error("timed TX fault: USB not idle before submit");
      m_timed_fault = true;
      return TxOutcome::Fault;
    }
    // The same 5 ms budget covers USB completion and the remaining NIC tail.
    // This is a laboratory bound until an independent RF witness measures it.
    constexpr std::uint32_t kTailReserveUs = 5000;
    const auto now_us = monotonic_us();
    if (!m_timed.can_tx(now_us, tx_class == TxClass::TimedControl,
                        *airtime + kTailReserveUs))
      return TxOutcome::GateClosed;
    const auto tx_before = m_radio->GetTxStats();
    if (!m_radio->send_packet(data, length)) {
      m_logger->error("timed TX fault: USB submit failed");
      m_timed_fault = true;
      return TxOutcome::Fault;
    }
    // Hopping has a full dwell to absorb normal USB jitter; fixed TDMA has
    // only a short data seat, where blocking every packet for 5 ms starves
    // video. Both modes still hold RF ownership until a late TX drains.
    const std::uint32_t normal_usb_wait_us = m_timed.hopping() ? 5000 : 2000;
    const auto submitted_us = monotonic_us();
    const bool usb_idle_on_time = m_radio->WaitTxIdle(normal_usb_wait_us);
    const auto tx_after = m_radio->GetTxStats();
    if (tx_after.failed != tx_before.failed) {
      m_logger->error(
          "timed TX fault: USB transfer failed; before={} after={} "
          "last_error_rc={} last_timeout={}",
          tx_before.failed, tx_after.failed,
          tx_after.last_error_rc, tx_after.last_was_timeout);
      m_timed_fault = true;
      return TxOutcome::Fault;
    }
    if (!usb_idle_on_time) {
      // Keep the RF mutex and TX gate closed until the outstanding transfer
      // drains. A late completion may lose a hop, but it must never retune
      // while the old-frequency packet is still pending. Retune already
      // tolerates up to 100 ms of drain backpressure; use the same total
      // bound here so a successful 25+ ms USB completion is not a fatal TX.
      constexpr std::uint32_t kMaxUsbDrainUs = 100000;
      const bool drained = m_radio->WaitTxIdle(
          kMaxUsbDrainUs - normal_usb_wait_us);
      if (!drained || m_radio->GetTxStats().failed != tx_before.failed) {
        m_logger->error(
            "timed TX fault: USB transfer did not drain safely after {} us",
            monotonic_us() - submitted_us);
        m_timed_fault = true;
        return TxOutcome::Fault;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(kTailReserveUs));
      m_logger->warn(
          "timed TX USB completion exceeded {} us; drained after {} us with RF held",
          normal_usb_wait_us, monotonic_us() - submitted_us);
      return TxOutcome::GateClosed;
    }
    return TxOutcome::Submitted;
  }

  bool timed_configure(const openhd_single_time::Plan& plan,
                       std::uint32_t generation) {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    for (std::uint32_t i = 0; i < plan.channel_count; ++i) {
      if (plan.frequencies_mhz[i] > 0xffff) return false;
      const auto channel = devourer::freq_to_chan(
          static_cast<std::uint16_t>(plan.frequencies_mhz[i]));
      if (channel <= 0 || channel > 255)
        return false;
    }
    if (!timed_chip_supported() || !m_initialized || m_width_mhz != 20 ||
        generation == 0 ||
        !m_radio->WaitTxIdle(10000) ||
        !m_radio->WaitMacTxIdle(10000) ||
        !m_timed.configure(plan, m_frequency_mhz)) return false;
    m_timed_generation = generation;
    m_timed_fault = false;
    if (!m_timed_thread.joinable()) {
      m_timed_thread_run = true;
      m_timed_thread = std::thread([this] { timed_loop(); });
    }
    return true;
  }

  bool timed_set_phase(std::uint64_t local_us, std::uint32_t phase_us) {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    return m_timed.set_phase(monotonic_us(), local_us, phase_us);
  }

  bool timed_set_peer_lease(std::uint64_t received_us) {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    return m_timed.set_peer_lease(monotonic_us(), received_us);
  }

  bool timed_set_members(const std::array<std::uint64_t, 4>& members) {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    return m_timed.set_members(monotonic_us(), members);
  }

  bool timed_wait_first() {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    return m_timed.wait_first();
  }

  openhd_single_time::Status timed_status() {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    auto result = m_timed.status(monotonic_us());
    if (!result.configured) result.target_frequency_mhz = m_frequency_mhz;
    if (m_timed_fault) {
      result.tx_gated = 1;
      result.state = openhd_single_time::RunState::Fault;
    }
    return result;
  }

  void timed_stop() {
    std::lock_guard<std::mutex> lock(m_timed_mutex);
    m_timed.stop();
    m_timed_generation = 0;
    // Stop and rejected replacement plans must not fall back to legacy TX.
    // A fresh client session or a successful Configure is needed to reopen TX.
    m_timed_fault = true;
  }

  std::uint32_t frequency_mhz() const { return m_frequency_mhz; }

  void set_identity(const Args& args) {
    m_vid = args.vid;
    m_pid = args.pid;
    m_bus = args.bus;
    m_port = args.port;
  }
  std::uint16_t vid() const { return m_vid; }
  std::uint16_t pid() const { return m_pid; }
  int bus() const { return m_bus; }
  const std::string& port() const { return m_port; }
  const devourer::AdapterCaps& caps() const { return m_caps; }
  bool timed_chip_supported() const {
    return m_caps.chip_name &&
           std::strcmp(m_caps.chip_name, "RTL8812A") == 0;
  }

  void set_rx_callback(std::function<void(const Packet&)> callback) {
    m_rx_callback = std::move(callback);
  }

  void close() {
    m_timed_thread_run = false;
    if (m_timed_thread.joinable()) m_timed_thread.join();
    timed_stop();
    stop_rx();
    m_radio.reset();  // quiesce USB TX while handle and context remain valid
    if (m_handle) {
      if (m_interface_claimed) {
        const int release = libusb_release_interface(m_handle, m_interface);
        if (release != 0)
          m_logger->warn("libusb_release_interface returned {}", release);
        m_interface_claimed = false;
      }
      if (m_kernel_driver_was_attached) {
        const int active = libusb_kernel_driver_active(m_handle, m_interface);
        if (active == 0) {
          const int attach = libusb_attach_kernel_driver(m_handle, m_interface);
          if (attach != 0 && attach != LIBUSB_ERROR_NOT_FOUND)
            m_logger->error("failed to reattach original kernel driver: {}", attach);
        }
      }
      libusb_close(m_handle);
      m_handle = nullptr;
    }
    m_usb_lock.reset();
    if (m_context) {
      libusb_exit(m_context);
      m_context = nullptr;
    }
    m_initialized = false;
  }

 private:
  void timed_loop() {
    std::uint64_t drain_wait_started_us = 0;
    while (m_timed_thread_run) {
      {
        std::lock_guard<std::mutex> lock(m_timed_mutex);
        const auto now_us = monotonic_us();
        // A TX or retune fault stays closed until a fresh plan and phase.
        // Never keep retuning from a stale phase.
        const auto request = m_timed_fault ? std::nullopt : m_timed.step(now_us);
        if (!request) drain_wait_started_us = 0;
        if (request && m_radio && m_initialized) {
          bool success = false;
          const auto channel = devourer::freq_to_chan(
              static_cast<std::uint16_t>(request->frequency_mhz));
          // The mutex prevents new submissions while both USB and the MAC TX
          // state drain. Independent RF timing remains a release gate.
          const bool usb_idle = m_radio->WaitTxIdle(2000);
          const bool mac_idle = usb_idle && m_radio->WaitMacTxIdle(5000);
          if (!mac_idle) {
            // A single 5 ms MAC drain miss is not a hardware fault. Leave the
            // old slot active: Scheduler::status() then keeps TX gated while
            // the next loop retries. A stuck queue still faults within 100 ms.
            if (!drain_wait_started_us) {
              drain_wait_started_us = now_us;
              m_logger->warn("timed retune waiting for drain: usb_idle={} slot={} frequency={}",
                             usb_idle, request->slot, request->frequency_mhz);
            }
            if (monotonic_us() - drain_wait_started_us < 100000U)
              continue;
            m_logger->error("timed retune drain fault after 100 ms: usb_idle={} slot={} frequency={}",
                            usb_idle, request->slot, request->frequency_mhz);
            m_timed.retune_complete(*request, monotonic_us(), false);
            m_timed_fault = true;
            drain_wait_started_us = 0;
            continue;
          }
          drain_wait_started_us = 0;
          if (mac_idle && channel > 0 && channel <= 255) {
            try {
              m_radio->FastRetune(static_cast<std::uint8_t>(channel), true);
              m_frequency_mhz = request->frequency_mhz;
              success = true;
            } catch (const std::exception& error) {
              m_logger->error("timed retune failed: {}", error.what());
            } catch (...) {
              m_logger->error("timed retune failed with unknown exception");
            }
          }
          m_timed.retune_complete(*request, monotonic_us(), success);
          if (!success) m_timed_fault = true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void stop_rx() {
    if (m_radio && m_rx_running.load()) m_radio->StopRxLoop();
    if (m_rx_thread.joinable()) m_rx_thread.join();
    m_rx_running = false;
  }

  void on_rx_packet(const Packet& packet) {
    if (m_rx_callback &&
        packet.RxAtrib.pkt_rpt_type == RX_PACKET_TYPE::NORMAL_RX &&
        !packet.Data.empty() && packet.Data.size() + 1 <= kMaxPayload) {
      m_rx_callback(packet);
    }
  }

  Logger_t m_logger;
  libusb_context* m_context = nullptr;
  libusb_device_handle* m_handle = nullptr;
  int m_interface = 0;
  bool m_kernel_driver_was_attached = false;
  bool m_interface_claimed = false;
  bool m_initialized = false;
  std::atomic<bool> m_rx_running{false};
  std::thread m_rx_thread;
  std::shared_ptr<devourer::UsbDeviceLock> m_usb_lock;
  std::unique_ptr<IRadio> m_radio;
  devourer::AdapterCaps m_caps{};
  std::function<void(const Packet&)> m_rx_callback;
  std::uint16_t m_vid = 0;
  std::uint16_t m_pid = 0;
  int m_bus = 0;
  std::string m_port;
  std::uint32_t m_frequency_mhz = 0;
  std::uint32_t m_width_mhz = 0;
  std::mutex m_timed_mutex;
  openhd_single_time::Scheduler m_timed;
  bool m_timed_fault = false;
  std::uint32_t m_timed_generation = 0;
  std::atomic<bool> m_timed_thread_run{false};
  std::thread m_timed_thread;
};

class Service {
 public:
  Service(Args args, Logger_t logger)
      : m_args(std::move(args)), m_logger(std::move(logger)) {}
  ~Service() {
    if (m_listener >= 0) ::close(m_listener);
    if (m_bound_socket) ::unlink(m_args.socket_path.c_str());
    if (m_radio_access_attempted) restore_kernel_binding();
  }

  int run() {
    if (!listen_socket()) return 2;
    m_logger->info("listening at {} for {:04x}:{:04x} bus={} port={}",
                   m_args.socket_path, m_args.vid, m_args.pid, m_args.bus,
                   m_args.port);
    while (!g_devourer_should_stop) {
      pollfd descriptor{m_listener, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, 250);
      if (ready < 0) {
        if (errno == EINTR) continue;
        m_logger->error("listener poll failed: {}", errno);
        return 2;
      }
      if (ready == 0) continue;
      const int client = ::accept4(m_listener, nullptr, nullptr, SOCK_CLOEXEC);
      if (client < 0) {
        if (errno == EINTR || errno == EAGAIN) continue;
        m_logger->error("accept failed: {}", errno);
        continue;
      }
      timeval send_timeout{1, 0};
      (void)::setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &send_timeout,
                         sizeof(send_timeout));
      m_logger->info("OpenHD client connected");
      serve_client(client);
      ::close(client);
      m_logger->info("OpenHD client disconnected; USB session released");
    }
    return 0;
  }

 private:
  void restore_kernel_binding() {
    // A previous service process may have been killed after detaching the
    // kernel driver. The replacement session then cannot know that the driver
    // was originally attached. Restore it only as this service exits, rather
    // than between OpenHD client sessions during automatic recovery.
    libusb_context* context = nullptr;
    if (libusb_init(&context) != 0) return;
    libusb_device_handle* handle = open_device(context, m_args, m_logger);
    if (handle) {
      const int iface = devourer::find_wifi_interface(handle);
      const int active = libusb_kernel_driver_active(handle, iface);
      if (active == 0) {
        const int rc = libusb_attach_kernel_driver(handle, iface);
        if (rc == 0)
          m_logger->info("restored kernel driver on USB interface {}", iface);
        else if (rc != LIBUSB_ERROR_NOT_FOUND)
          m_logger->warn("kernel driver reattach failed on interface {}: {}",
                         iface, rc);
      } else if (active < 0 && active != LIBUSB_ERROR_NOT_SUPPORTED) {
        m_logger->warn("cannot check kernel driver on interface {}: {}",
                       iface, active);
      }
      libusb_close(handle);
    }
    libusb_exit(context);
  }

  bool listen_socket() {
    sockaddr_un address{};
    if (m_args.socket_path.size() >= sizeof(address.sun_path)) {
      m_logger->error("socket path is too long");
      return false;
    }
    struct stat existing {};
    if (::lstat(m_args.socket_path.c_str(), &existing) == 0) {
      if (!S_ISSOCK(existing.st_mode)) {
        m_logger->error("refusing to unlink a non-socket path: {}",
                        m_args.socket_path);
        return false;
      }
      const int probe = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
      if (probe < 0) {
        m_logger->error("cannot check existing radio socket: {}", errno);
        return false;
      }
      sockaddr_un probe_address{};
      probe_address.sun_family = AF_UNIX;
      std::memcpy(probe_address.sun_path, m_args.socket_path.c_str(),
                  m_args.socket_path.size() + 1);
      const int probe_result = ::connect(
          probe, reinterpret_cast<const sockaddr*>(&probe_address),
          sizeof(probe_address));
      const int probe_error = errno;
      ::close(probe);
      if (probe_result == 0) {
        m_logger->error("a radio service is already listening at {}",
                        m_args.socket_path);
        return false;
      }
      if (probe_error != ECONNREFUSED && probe_error != ENOENT) {
        m_logger->error("existing radio socket cannot be probed: {}",
                        probe_error);
        return false;
      }
      if (::unlink(m_args.socket_path.c_str()) != 0) {
        m_logger->error("cannot remove stale radio socket: {}", errno);
        return false;
      }
    } else if (errno != ENOENT) {
      m_logger->error("cannot inspect radio socket path: {}", errno);
      return false;
    }
    m_listener = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (m_listener < 0) {
      m_logger->error("cannot create Unix socket: {}", errno);
      return false;
    }
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, m_args.socket_path.c_str(),
                m_args.socket_path.size() + 1);
    if (::bind(m_listener, reinterpret_cast<const sockaddr*>(&address),
               sizeof(address)) != 0 ||
        ::chmod(m_args.socket_path.c_str(), 0660) != 0 ||
        ::listen(m_listener, 1) != 0) {
      m_logger->error("cannot bind/listen at {}: {}", m_args.socket_path,
                      errno);
      return false;
    }
    m_bound_socket = true;
    return true;
  }

  bool send_message(int fd, std::mutex& send_mutex, std::uint16_t type,
                    std::uint32_t sequence, const std::uint8_t* payload,
                    std::size_t payload_size, bool nonblocking = false) {
    const auto packet = encode(type, sequence, payload, payload_size);
    if (packet.empty()) return false;
    std::lock_guard<std::mutex> lock(send_mutex);
    const int flags = MSG_NOSIGNAL | (nonblocking ? MSG_DONTWAIT : 0);
    const ssize_t sent = ::send(fd, packet.data(), packet.size(), flags);
    return sent == static_cast<ssize_t>(packet.size());
  }

  bool send_response(int fd, std::mutex& send_mutex, std::uint32_t sequence,
                     std::uint16_t request_type, std::uint16_t result_code,
                     int native_error, const std::string& detail,
                     bool readback_verified = false,
                     const std::vector<std::uint8_t>& extra = {}) {
    const auto text_size = std::min<std::size_t>(detail.size(), 512);
    std::vector<std::uint8_t> payload;
    put_u16(payload, request_type);
    put_u16(payload, result_code);
    put_i32(payload, native_error);
    put_u8(payload, readback_verified ? 1 : 0);
    put_u8(payload, 0);
    put_u16(payload, static_cast<std::uint16_t>(text_size));
    payload.insert(payload.end(), detail.begin(), detail.begin() + text_size);
    payload.insert(payload.end(), extra.begin(), extra.end());
    return send_message(fd, send_mutex, kResponse, sequence, payload.data(),
                        payload.size());
  }

  bool send_ready(int fd, std::mutex& send_mutex, std::uint32_t sequence,
                  const RadioSession& session) {
    std::vector<std::uint8_t> payload;
    put_u16(payload, session.vid());
    put_u16(payload, session.pid());
    put_u8(payload, static_cast<std::uint8_t>(session.bus()));
    put_u8(payload, static_cast<std::uint8_t>(session.port().size()));
    payload.insert(payload.end(), session.port().begin(), session.port().end());
    put_u8(payload, 0);  // EFUSE MAC omitted until a reliable API is available
    const char* model = session.caps().marketing_names;
    if (!model || !*model) model = session.caps().chip_name;
    const std::string model_text = model ? model : "RTL8812AU";
    const std::string chip_text =
        session.caps().chip_name ? session.caps().chip_name : "RTL8812AU";
    put_u8(payload, static_cast<std::uint8_t>(model_text.size()));
    payload.insert(payload.end(), model_text.begin(), model_text.end());
    put_u8(payload, static_cast<std::uint8_t>(chip_text.size()));
    payload.insert(payload.end(), chip_text.begin(), chip_text.end());
    std::uint16_t capabilities = kCapabilityFixedRf;
    if (session.caps().txpwr.supported)
      capabilities |= kCapabilityTxPowerIndex;
    if (session.timed_chip_supported()) {
      if (kTimedModes & 1U) capabilities |= kCapabilityFixedTdma;
      if (kTimedModes & 2U) capabilities |= kCapabilityFhss;
      if (kTimedModes & 4U) capabilities |= kCapabilityFhssTdma;
    }
    put_u16(payload, capabilities);
    return send_message(fd, send_mutex, kReady, sequence, payload.data(),
                        payload.size());
  }

  bool receive_message(int fd, std::array<std::uint8_t, kMaxMessage>& buffer,
                       Message& message) {
    const ssize_t received = ::recv(fd, buffer.data(), buffer.size(), 0);
    if (received <= 0) return false;
    return decode(buffer.data(), static_cast<std::size_t>(received), message);
  }

  void serve_client(int client) {
    std::mutex send_mutex;
    std::array<std::uint8_t, kMaxMessage> buffer{};
    Message message;
    if (!receive_message(client, buffer, message) || message.type != kHello ||
        message.sequence != 1 || message.payload_size != 0) {
      m_logger->warn("client sent an invalid IPC handshake");
      return;
    }

    RadioSession session(m_logger);
    session.set_identity(m_args);
    m_radio_access_attempted = true;
    std::string open_error;
    if (!session.open(m_args, open_error)) {
      const std::string detail = open_error.empty()
                                     ? "failed to claim or initialize the selected USB radio"
                                     : open_error;
      (void)send_message(client, send_mutex, kDeviceError, 0,
                         reinterpret_cast<const std::uint8_t*>(detail.data()),
                         detail.size());
      return;
    }
    session.set_rx_callback([&](const Packet& packet) {
      std::vector<std::uint8_t> payload;
      payload.reserve(packet.Data.size() + 1);
      std::uint8_t flags = packet.RxAtrib.fcs_present ? kRadiotapFcs : 0;
      if (packet.RxAtrib.crc_err) flags |= kRadiotapBadFcs;
      put_u8(payload, flags);
      payload.insert(payload.end(), packet.Data.begin(), packet.Data.end());
      const auto rx_sequence = m_rx_sequence++;
      const bool forwarded = send_message(
          client, send_mutex, kRxPacket, rx_sequence, payload.data(),
          payload.size(), true);
      record_trace_rx(packet, rx_sequence);
      record_rx(packet, forwarded);
    });
    if (!send_ready(client, send_mutex, message.sequence, session)) {
      m_logger->warn("failed to send Devourer radio identity");
      return;
    }

    while (!g_devourer_should_stop) {
      pollfd descriptor{client, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, 250);
      if (ready < 0) {
        if (errno == EINTR) continue;
        break;
      }
      if (ready == 0) continue;
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) break;
      if (!receive_message(client, buffer, message)) break;
      std::size_t offset = 0;
      std::string detail;
      if (message.type == kSetFixedRf && message.payload_size == 9) {
        std::uint32_t frequency = 0;
        std::uint32_t width = 0;
        const bool decoded = get_u32(message.payload, message.payload_size,
                                     offset, frequency) &&
                             get_u32(message.payload, message.payload_size,
                                     offset, width) &&
                             offset < message.payload_size;
        const std::uint8_t primary_channel_offset =
            decoded ? message.payload[offset] : 0;
        const bool applied = decoded &&
                             session.set_fixed_rf(frequency, width,
                                                  primary_channel_offset,
                                                  detail);
        (void)send_response(client, send_mutex, message.sequence, message.type,
                            applied ? 0 : 5, applied ? 0 : EINVAL,
                            applied ? "fixed RF profile applied" : detail);
      } else if (message.type == kSetTxPower &&
                 message.payload_size == 5) {
        std::uint32_t raw_power = 0;
        if (!get_u32(message.payload, message.payload_size, offset, raw_power)) {
          break;
        }
        const auto power = static_cast<std::int32_t>(raw_power);
        const bool applied = session.set_tx_power(power, detail);
        (void)send_response(client, send_mutex, message.sequence, message.type,
                            applied ? 0 : 5, applied ? 0 : EINVAL,
                            applied ? "TX power index applied" : detail);
      } else if ((message.type == kTxPacket ||
                  message.type == kTimedControlBeacon ||
                  message.type == kTimedDataPacket) &&
                 message.payload_size > sizeof(std::uint32_t)) {
        std::uint32_t tx_sequence = 0;
        if (!get_u32(message.payload, message.payload_size, offset,
                     tx_sequence)) {
          break;
        }
        std::uint32_t generation = 0;
        if (message.type != kTxPacket &&
            !get_u32(message.payload, message.payload_size, offset,
                     generation)) {
          break;
        }
        const auto* tx_data = message.payload + offset;
        const auto tx_length = message.payload_size - offset;
        record_trace("TX", tx_sequence, tx_data, tx_length, "");
        log_tx_prefix(tx_data, tx_length, tx_sequence);
        const auto tx_class = message.type == kTimedControlBeacon
            ? RadioSession::TxClass::TimedControl
            : message.type == kTimedDataPacket
                ? RadioSession::TxClass::TimedData
                : RadioSession::TxClass::LegacyData;
        const auto outcome = session.send_packet(
            tx_data, tx_length, tx_class, generation);
        {
          std::lock_guard<std::mutex> lock(m_stats_mutex);
          ++m_tx_attempted;
          if (outcome == RadioSession::TxOutcome::Submitted) {
            ++m_tx_submitted;
          } else {
            ++m_tx_rejected;
          }
        }
        if (outcome == RadioSession::TxOutcome::Fault) {
          const std::string error = "Devourer rejected TX sequence " +
                                    std::to_string(tx_sequence);
          (void)send_message(client, send_mutex, kTxError, message.sequence,
                             reinterpret_cast<const std::uint8_t*>(error.data()),
                             error.size(), true);
        }
      } else if (message.type == kSingleTimeControl &&
                 message.payload_size >= 1) {
        const std::uint8_t operation = message.payload[0];
        bool applied = false;
        bool malformed = false;
        std::size_t position = 1;
        if (operation == 7 && message.payload_size == 1) {
          const auto current = session.timed_status();
          std::vector<std::uint8_t> status;
          for (const std::uint32_t value :
               {current.configured, current.synchronized, current.tx_gated,
                current.target_frequency_mhz, current.slot,
                static_cast<std::uint32_t>(current.state)})
            put_u32(status, value);
          (void)send_response(client, send_mutex, message.sequence,
                              message.type, 0, 0, "single-time status",
                              false, status);
        } else {
          openhd_single_time::Plan plan{};
          std::uint32_t generation = 0;
          if (operation == 1) {
            // A replacement Configure revokes the previous generation even
            // when the new plan is malformed or its mode is not enabled.
            session.timed_stop();
            malformed = !decode_single_time_plan(
                message.payload, message.payload_size, plan, generation);
            if (!malformed &&
                (kTimedModes & timed_mode_bit(plan.flags)) != 0)
              applied = session.timed_configure(plan, generation);
          } else if (operation == 2 && message.payload_size == 13) {
            std::uint64_t local_us = 0;
            std::uint32_t phase_us = 0;
            malformed = !get_u64(message.payload, message.payload_size,
                                 position, local_us) ||
                        !get_u32(message.payload, message.payload_size,
                                 position, phase_us);
            if (!malformed) applied = session.timed_set_phase(local_us, phase_us);
          } else if (operation == 3 && message.payload_size == 9) {
            std::uint64_t received_us = 0;
            malformed = !get_u64(message.payload, message.payload_size,
                                 position, received_us);
            if (!malformed) applied = session.timed_set_peer_lease(received_us);
          } else if (operation == 4 && message.payload_size == 33) {
            std::array<std::uint64_t, 4> members{};
            for (auto& bits : members)
              malformed |= !get_u64(message.payload, message.payload_size,
                                    position, bits);
            if (!malformed) applied = session.timed_set_members(members);
          } else if (operation == 5 && message.payload_size == 1) {
            applied = session.timed_wait_first();
          } else if (operation == 6 && message.payload_size == 1) {
            session.timed_stop();
            applied = true;
          } else {
            malformed = true;
          }
          (void)send_response(
              client, send_mutex, message.sequence, message.type,
              malformed ? 5 : (applied ? 0 : 2),
              malformed ? EINVAL : (applied ? 0 : ENOTSUP),
              malformed ? "invalid single-time command" :
                          (applied ? "single-time command applied" :
                                     "Devourer timed executor is not enabled"));
        }
      } else if (message.type == kStop && message.payload_size == 0) {
        (void)send_response(client, send_mutex, message.sequence, message.type,
                            0, 0, "radio session stopped");
        break;
      } else {
        detail = "unsupported or malformed radio IPC request";
        (void)send_response(client, send_mutex, message.sequence, message.type,
                            5, EPROTO, detail);
      }
    }
    session.close();
    log_session_stats();
    write_trace_records();
  }

  struct TraceRecord {
    const char* direction = "";
    std::uint32_t sequence = 0;
    std::uint64_t monotonic_ns = 0;
    std::string metadata;
    std::vector<std::uint8_t> bytes;
  };

  void record_trace(const char* direction, std::uint32_t sequence,
                    const std::uint8_t* data, std::size_t length,
                    std::string metadata) {
    constexpr std::size_t kMaxTraceRecords = 512;
    constexpr std::size_t kMaxTraceBytes = 2 * 1024 * 1024;
    if (m_args.trace_file.empty() || !data || length == 0) return;
    std::lock_guard<std::mutex> lock(m_trace_mutex);
    if (m_trace_records.size() >= kMaxTraceRecords ||
        length > kMaxTraceBytes - m_trace_bytes) {
      ++m_trace_dropped;
      return;
    }
    TraceRecord record;
    record.direction = direction;
    record.sequence = sequence;
    record.monotonic_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    record.metadata = std::move(metadata);
    record.bytes.assign(data, data + length);
    m_trace_bytes += length;
    m_trace_records.emplace_back(std::move(record));
  }

  void record_trace_rx(const Packet& packet, std::uint32_t sequence) {
    const std::string metadata =
        "fcs_present=" + std::to_string(packet.RxAtrib.fcs_present) +
        " crc_err=" + std::to_string(packet.RxAtrib.crc_err) +
        " phy_status=" + std::to_string(packet.RxAtrib.physt) +
        " report_type=" +
        std::to_string(static_cast<unsigned>(packet.RxAtrib.pkt_rpt_type));
    record_trace("RX", sequence, packet.Data.data(), packet.Data.size(),
                 metadata);
  }

  void write_trace_records() {
    if (m_args.trace_file.empty()) return;
    std::vector<TraceRecord> records;
    std::size_t dropped = 0;
    {
      std::lock_guard<std::mutex> lock(m_trace_mutex);
      records.swap(m_trace_records);
      dropped = m_trace_dropped;
      m_trace_bytes = 0;
      m_trace_dropped = 0;
    }
    if (records.empty() && dropped == 0) return;
    std::ofstream trace(m_args.trace_file, std::ios::out | std::ios::app);
    if (!trace) {
      m_logger->warn("cannot append packet trace to {}", m_args.trace_file);
      return;
    }
    static constexpr char kHex[] = "0123456789abcdef";
    for (const auto& record : records) {
      trace << record.direction << " seq=" << record.sequence
            << " monotonic_ns=" << record.monotonic_ns
            << " len=" << record.bytes.size();
      if (!record.metadata.empty()) trace << ' ' << record.metadata;
      trace << " hex=";
      for (const auto byte : record.bytes) {
        trace << kHex[byte >> 4] << kHex[byte & 0x0f];
      }
      trace << '\n';
    }
    trace << "TRACE_SUMMARY records=" << records.size()
          << " dropped=" << dropped << '\n';
    m_logger->info("wrote {} packet trace records to {} (dropped={})",
                   records.size(), m_args.trace_file, dropped);
  }

  void record_rx(const Packet& packet, bool forwarded) {
    std::string source = "short-frame";
    if (packet.Data.size() >= 16) {
      char address[18]{};
      std::snprintf(address, sizeof(address),
                    "%02x:%02x:%02x:%02x:%02x:%02x", packet.Data[10],
                    packet.Data[11], packet.Data[12], packet.Data[13],
                    packet.Data[14], packet.Data[15]);
      source = address;
    }
    std::lock_guard<std::mutex> lock(m_stats_mutex);
    if (forwarded) {
      ++m_rx_forwarded;
    } else {
      ++m_rx_dropped;
    }
    if (packet.RxAtrib.crc_err) ++m_rx_bad_fcs;
    if (packet.RxAtrib.physt) ++m_rx_with_phy_status;
    ++m_rx_source_counts[source];
  }

  void log_tx_prefix(const std::uint8_t* data, std::size_t length,
                     std::uint32_t sequence) {
    if (!data || m_tx_logged >= 8) return;
    const auto prefix_length = std::min<std::size_t>(length, 64);
    char prefix[64 * 2 + 1]{};
    for (std::size_t i = 0; i < prefix_length; ++i) {
      std::snprintf(prefix + i * 2, 3, "%02x", data[i]);
    }
    const auto radiotap_length =
        length >= 4 ? static_cast<std::size_t>(data[2] | (data[3] << 8)) : 0;
    m_logger->info(
        "OpenHD TX request #{} len={} radiotap_len={} prefix64={}",
        sequence, length, radiotap_length, prefix);
    ++m_tx_logged;
  }

  void log_session_stats() {
    std::map<std::string, std::uint64_t> source_counts;
    std::uint64_t tx_attempted = 0;
    std::uint64_t tx_submitted = 0;
    std::uint64_t tx_rejected = 0;
    std::uint64_t rx_forwarded = 0;
    std::uint64_t rx_dropped = 0;
    std::uint64_t rx_bad_fcs = 0;
    std::uint64_t rx_with_phy_status = 0;
    {
      std::lock_guard<std::mutex> lock(m_stats_mutex);
      source_counts = m_rx_source_counts;
      tx_attempted = m_tx_attempted;
      tx_submitted = m_tx_submitted;
      tx_rejected = m_tx_rejected;
      rx_forwarded = m_rx_forwarded;
      rx_dropped = m_rx_dropped;
      rx_bad_fcs = m_rx_bad_fcs;
      rx_with_phy_status = m_rx_with_phy_status;
      m_rx_source_counts.clear();
      m_tx_attempted = 0;
      m_tx_submitted = 0;
      m_tx_rejected = 0;
      m_tx_logged = 0;
      m_rx_forwarded = 0;
      m_rx_dropped = 0;
      m_rx_bad_fcs = 0;
      m_rx_with_phy_status = 0;
    }

    m_logger->info(
        "radio counters: TX attempted={} submit_ok={} submit_rejected={} "
        "RX forwarded={} dropped={} bad_fcs={} phy_status_frames={}",
        tx_attempted, tx_submitted, tx_rejected, rx_forwarded, rx_dropped,
        rx_bad_fcs, rx_with_phy_status);
    std::vector<std::pair<std::string, std::uint64_t>> ranked_sources(
        source_counts.begin(), source_counts.end());
    std::sort(ranked_sources.begin(), ranked_sources.end(),
              [](const auto& left, const auto& right) {
                if (left.second != right.second)
                  return left.second > right.second;
                return left.first < right.first;
              });
    std::ostringstream summary;
    const auto limit = std::min<std::size_t>(ranked_sources.size(), 32);
    for (std::size_t i = 0; i < limit; ++i) {
      if (i) summary << ",";
      summary << ranked_sources[i].first << ":" << ranked_sources[i].second;
    }
    m_logger->info("RX source addr2 top32={}", summary.str());
  }

  Args m_args;
  Logger_t m_logger;
  int m_listener = -1;
  bool m_bound_socket = false;
  bool m_radio_access_attempted = false;
  std::uint32_t m_rx_sequence = 1;
  std::mutex m_stats_mutex;
  std::uint64_t m_tx_attempted = 0;
  std::uint64_t m_tx_submitted = 0;
  std::uint64_t m_tx_rejected = 0;
  std::uint64_t m_tx_logged = 0;
  std::uint64_t m_rx_forwarded = 0;
  std::uint64_t m_rx_dropped = 0;
  std::uint64_t m_rx_bad_fcs = 0;
  std::uint64_t m_rx_with_phy_status = 0;
  std::map<std::string, std::uint64_t> m_rx_source_counts;
  std::mutex m_trace_mutex;
  std::vector<TraceRecord> m_trace_records;
  std::size_t m_trace_bytes = 0;
  std::size_t m_trace_dropped = 0;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::strcmp(argv[1], "--print-lab-modes") == 0) {
    std::printf("%d\n", OPENHD_TIMED_LAB_MODES);
    return 0;
  }
  Args args;
  if (!parse_args(argc, argv, args)) {
    std::fprintf(stderr,
                 "usage: openhd-radio-service --socket PATH --vid N --pid N "
                 "--bus N --port N[.N...] [--trace-file PATH]\n");
    return 2;
  }
  auto logger = std::make_shared<Logger>();
  install_devourer_signal_handlers();
  Service service(std::move(args), logger);
  return service.run();
}
