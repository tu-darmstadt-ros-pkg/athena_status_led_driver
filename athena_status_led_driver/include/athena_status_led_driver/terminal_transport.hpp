#ifndef ATHENA_STATUS_LED_DRIVER_TERMINAL_TRANSPORT_HPP
#define ATHENA_STATUS_LED_DRIVER_TERMINAL_TRANSPORT_HPP

#include "athena_status_led_driver/led_transport.hpp"

#include <string>
#include <vector>

namespace athena_status_led_driver
{

/**
 * @brief Transport that renders the LED ring in the terminal using ANSI 24-bit colors.
 *
 * Arranges LEDs in a circular layout and draws each as a colored Unicode circle.
 * Designed for local development and visual testing without hardware.
 */
class TerminalTransport : public LedTransport
{
public:


  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool send(const std::vector<Color>& leds) override;

private:
  bool open_ = false;
};

}  // namespace athena_status_led_driver

#endif  // ATHENA_STATUS_LED_DRIVER_TERMINAL_TRANSPORT_HPP
