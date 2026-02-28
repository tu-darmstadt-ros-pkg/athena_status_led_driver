#ifndef ATHENA_STATUS_LED_DRIVER_LED_RING_CONTROLLER_HPP
#define ATHENA_STATUS_LED_DRIVER_LED_RING_CONTROLLER_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include "athena_status_led_driver/led_transport.hpp"

#include <memory>
#include <vector>

namespace athena_status_led_driver
{

/**
 * @brief Manages the LED effect pipeline and transport.
 *
 * Effects are rendered in the order they were added.
 * Typically: fill effects first (rainbow, operating mode), then blend effects
 * (power supply chase, battery pulse, spot light).
 * Later fill effects overwrite earlier ones by writing all pixels.
 * Global brightness is applied after all effects.
 */
class LedRingController
{
public:
  LedRingController( size_t led_count, std::shared_ptr<LedTransport> transport );

  /// Add an effect. Effects are rendered in the order they are added.
  void addEffect( std::shared_ptr<LedEffect> effect );

  /// Set global brightness (0.0–1.0)
  void setBrightness( float brightness );

  float brightness() const;

  /// Update all effects, render the frame, and send it
  void tick( double dt );

  /// Get current pixel buffer (for testing)
  const std::vector<Color> &pixels() const;

  /// Get the transport (for testing)
  std::shared_ptr<LedTransport> transport() const;

private:
  size_t led_count_;
  std::shared_ptr<LedTransport> transport_;
  std::vector<Color> pixels_;
  std::vector<std::shared_ptr<LedEffect>> effects_;
  float brightness_ = 1.0f;
};

} // namespace athena_status_led_driver

#endif // ATHENA_STATUS_LED_DRIVER_LED_RING_CONTROLLER_HPP
