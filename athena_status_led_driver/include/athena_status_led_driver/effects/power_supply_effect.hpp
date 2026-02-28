#ifndef ATHENA_STATUS_LED_DRIVER_POWER_SUPPLY_EFFECT_HPP
#define ATHENA_STATUS_LED_DRIVER_POWER_SUPPLY_EFFECT_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include <cmath>

namespace athena_status_led_driver
{

/**
 * @brief Overlay effect that shows 4 green LEDs circling when on power supply.
 *
 * Active when power_source == POWER_SUPPLY (0). The 4 LEDs smoothly rotate
 * around the ring using fractional positions for smooth animation.
 */
class PowerSupplyEffect : public LedEffect
{
public:
  static constexpr int NUM_LED_GROUPS = 4;
  static constexpr int NUM_LEDS_PER_GROUP = 4;
  static constexpr double ROTATION_SPEED = 0.25; // revolutions per second

  explicit PowerSupplyEffect( size_t led_count ) : led_count_( led_count ) { }

  bool isActive() const override { return on_power_supply_; }

  void update( double dt ) override;
  void render( std::vector<Color> &pixels ) override;

  /// Set power source state
  void setOnPowerSupply( bool on_power_supply )
  {
    on_power_supply_ = on_power_supply;
    if ( !on_power_supply )
      position_ = 0.0;
  }

  /// Get current position (for testing)
  double position() const { return position_; }

private:
  size_t led_count_;
  bool on_power_supply_ = false;
  double position_ = 0.0;
};

} // namespace athena_status_led_driver

#endif // ATHENA_STATUS_LED_DRIVER_POWER_SUPPLY_EFFECT_HPP
