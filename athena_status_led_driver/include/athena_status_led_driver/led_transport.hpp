#ifndef ATHENA_STATUS_LED_DRIVER_LED_TRANSPORT_HPP
#define ATHENA_STATUS_LED_DRIVER_LED_TRANSPORT_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include <vector>

namespace athena_status_led_driver
{

/**
 * @brief Abstract interface for sending pixel data to the LED strip.
 *
 * Concrete implementations: SpiTransport (hardware), MockTransport (testing).
 */
class LedTransport
{
public:
  virtual ~LedTransport() = default;

  /// Open the transport (e.g., open SPI device). Returns true on success.
  virtual bool open() = 0;

  /// Close the transport.
  virtual void close() = 0;

  /// Whether the transport is currently open
  virtual bool isOpen() const = 0;

  /// Send a frame of LED colors. Returns true on success.
  virtual bool send(const std::vector<Color>& leds) = 0;
};

}  // namespace athena_status_led_driver

#endif  // ATHENA_STATUS_LED_DRIVER_LED_TRANSPORT_HPP
