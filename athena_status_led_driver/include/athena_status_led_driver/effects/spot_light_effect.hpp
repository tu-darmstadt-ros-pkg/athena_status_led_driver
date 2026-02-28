#ifndef ATHENA_STATUS_LED_DRIVER_SPOT_LIGHT_EFFECT_HPP
#define ATHENA_STATUS_LED_DRIVER_SPOT_LIGHT_EFFECT_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include <cmath>

namespace athena_status_led_driver
{

/**
 * @brief Overlay effect for directional white illumination (spot light).
 *
 * When enabled, LEDs within a directional arc (center angle ± width/2)
 * are set to white at the specified brightness. LEDs outside the arc
 * are unaffected (base effect shows through).
 *
 * Setting width to 360° illuminates the entire ring.
 */
class SpotLightEffect : public LedEffect
{
public:
  explicit SpotLightEffect( size_t led_count ) : led_count_( led_count ) { }

  bool isActive() const override { return enabled_; }

  void update( double /*dt*/ ) override { }

  void render( std::vector<Color> &pixels ) override;

  /// Configure the spot light
  void setSpotLight( bool enable, float brightness, float direction_deg, float width_deg )
  {
    enabled_ = enable;
    brightness_ = std::clamp( brightness, 0.0f, 1.0f );
    direction_deg_ = direction_deg;
    width_deg_ = std::clamp( width_deg, 0.0f, 360.0f );
  }

  bool isEnabled() const { return enabled_; }
  float brightness() const { return brightness_; }
  float directionDeg() const { return direction_deg_; }
  float widthDeg() const { return width_deg_; }

private:
  /**
   * @brief Compute the alpha for a given LED index based on the directional arc.
   *
   * Returns 1.0 if the LED is inside the arc, with a smooth falloff at the edges
   * (over ~1 LED width) for anti-aliasing.
   */
  float ledAlpha( size_t led_index ) const;

  size_t led_count_;
  bool enabled_ = false;
  float brightness_ = 1.0f;
  float direction_deg_ = 0.0f;
  float width_deg_ = 360.0f;
};

} // namespace athena_status_led_driver

#endif // ATHENA_STATUS_LED_DRIVER_SPOT_LIGHT_EFFECT_HPP
