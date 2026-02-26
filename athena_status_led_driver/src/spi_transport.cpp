#include "athena_status_led_driver/spi_transport.hpp"

#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdexcept>

namespace athena_status_led_driver
{

static constexpr uint32_t PACKING = WS2812B_PACKING_SINGLE;
static constexpr uint32_t PREFIX_LEN = 1;
static constexpr uint32_t SUFFIX_LEN = 4;

SpiTransport::SpiTransport(const std::string& device, uint32_t speed_hz, uint32_t led_count)
  : device_(device), speed_hz_(speed_hz), led_count_(led_count)
{
  ws_leds_.resize(led_count);
  size_t buf_len = WS2812B_REQUIRED_BUFFER_LEN(led_count, PACKING, PREFIX_LEN, SUFFIX_LEN);
  spi_buffer_.resize(buf_len, 0);
}

SpiTransport::~SpiTransport()
{
  close();
}

bool SpiTransport::open()
{
  if (fd_ >= 0)
    return true;

  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0)
  {
    last_error_ = std::string("Failed to open ") + device_ + ": " + strerror(errno) +
                  " (errno=" + std::to_string(errno) + ")";
    return false;
  }

  // Configure SPI — helper to reduce repetitive error handling
  auto spiIoctl = [this](unsigned long request, void* arg, const char* name) -> bool {
    if (ioctl(fd_, request, arg) < 0)
    {
      last_error_ = std::string("ioctl ") + name + " failed: " + strerror(errno);
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  };

  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  uint32_t speed = speed_hz_;

  if (!spiIoctl(SPI_IOC_WR_MODE, &mode, "SPI_IOC_WR_MODE") ||
      !spiIoctl(SPI_IOC_WR_BITS_PER_WORD, &bits, "SPI_IOC_WR_BITS_PER_WORD") ||
      !spiIoctl(SPI_IOC_WR_MAX_SPEED_HZ, &speed, "SPI_IOC_WR_MAX_SPEED_HZ"))
    return false;

  // Send 4 zero bytes to ensure SDO starts low
  uint8_t zeros[4] = {0};
  struct spi_ioc_transfer xfer{};
  std::memset(&xfer, 0, sizeof(xfer));
  xfer.tx_buf = reinterpret_cast<uintptr_t>(zeros);
  xfer.len = 4;
  xfer.speed_hz = speed_hz_;
  xfer.bits_per_word = 8;
  ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer);

  // Configure ws2812b driver
  ws_handle_.config.packing = static_cast<ws2812b_packing_t>(PACKING);
  ws_handle_.config.pulse_len_1 = WS2812B_PULSE_LEN_6b;
  ws_handle_.config.pulse_len_0 = WS2812B_PULSE_LEN_2b;
  ws_handle_.config.first_bit_0 = WS2812B_FIRST_BIT_0_ENABLED;
  ws_handle_.config.prefix_len = PREFIX_LEN;
  ws_handle_.config.suffix_len = SUFFIX_LEN;
  ws_handle_.config.spi_bit_order = WS2812B_MSB_FIRST;

  ws_handle_.led_count = led_count_;
  ws_handle_.leds = ws_leds_.data();

  if (ws2812b_init(&ws_handle_) != 0)
  {
#ifndef WS2812B_DISABLE_ERROR_MSG
    last_error_ = std::string("ws2812b_init failed: ") + (ws2812b_error_msg ? ws2812b_error_msg : "unknown");
#else
    last_error_ = "ws2812b_init failed";
#endif
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  last_error_.clear();
  return true;
}

void SpiTransport::close()
{
  if (fd_ >= 0)
  {
    // Turn off all LEDs before closing
    std::memset(ws_leds_.data(), 0, ws_leds_.size() * sizeof(ws2812b_led_t));
    ws2812b_fill_buffer(&ws_handle_, spi_buffer_.data());

    struct spi_ioc_transfer xfer{};
    std::memset(&xfer, 0, sizeof(xfer));
    xfer.tx_buf = reinterpret_cast<uintptr_t>(spi_buffer_.data());
    xfer.len = static_cast<uint32_t>(spi_buffer_.size());
    xfer.speed_hz = speed_hz_;
    xfer.bits_per_word = 8;
    ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer);

    ::close(fd_);
    fd_ = -1;
  }
}

bool SpiTransport::isOpen() const
{
  return fd_ >= 0;
}

bool SpiTransport::send(const std::vector<Color>& leds)
{
  if (fd_ < 0)
    return false;

  // Copy colors into ws2812b LED array
  size_t count = std::min(leds.size(), static_cast<size_t>(led_count_));
  for (size_t i = 0; i < count; ++i)
  {
    ws_leds_[i].red = leds[i].r;
    ws_leds_[i].green = leds[i].g;
    ws_leds_[i].blue = leds[i].b;
  }

  // Fill the SPI buffer
  ws2812b_fill_buffer(&ws_handle_, spi_buffer_.data());

  // Transmit
  struct spi_ioc_transfer xfer{};
  std::memset(&xfer, 0, sizeof(xfer));
  xfer.tx_buf = reinterpret_cast<uintptr_t>(spi_buffer_.data());
  xfer.len = static_cast<uint32_t>(spi_buffer_.size());
  xfer.speed_hz = speed_hz_;
  xfer.bits_per_word = 8;

  int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer);
  return ret >= 0;
}

}  // namespace athena_status_led_driver
