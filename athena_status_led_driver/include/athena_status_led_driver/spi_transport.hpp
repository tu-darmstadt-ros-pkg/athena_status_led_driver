#ifndef ATHENA_STATUS_LED_DRIVER_SPI_TRANSPORT_HPP
#define ATHENA_STATUS_LED_DRIVER_SPI_TRANSPORT_HPP

#include "athena_status_led_driver/led_transport.hpp"

#include <cstdint>
#include <string>
#include <vector>

// Forward-declare the ws2812b handle (C library)
extern "C" {
#include "ws2812b.h"
}

namespace athena_status_led_driver
{

/**
 * @brief Concrete transport that sends LED data via Linux spidev + ws2812b_spi.
 *
 * Targets Radxa Zero 3E: SPI3_MOSI_M1 on PIN 19 → /dev/spidev3.0.
 */
class SpiTransport : public LedTransport
{
public:
  /**
   * @param device      spidev path, e.g., "/dev/spidev3.0"
   * @param speed_hz    SPI clock frequency (~6500000 for single packing)
   * @param led_count   number of LEDs on the strip
   */
  SpiTransport( const std::string &device, uint32_t speed_hz, uint32_t led_count );
  ~SpiTransport() override;

  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool send( const std::vector<Color> &leds ) override;

  /// Diagnostic: reason for the last open() failure
  const std::string &lastError() const { return last_error_; }

private:
  std::string device_;
  uint32_t speed_hz_;
  uint32_t led_count_;
  int fd_ = -1;
  std::string last_error_;

  ws2812b_handle_t ws_handle_{};
  std::vector<ws2812b_led_t> ws_leds_;
  std::vector<uint8_t> spi_buffer_;
};

} // namespace athena_status_led_driver

#endif // ATHENA_STATUS_LED_DRIVER_SPI_TRANSPORT_HPP
