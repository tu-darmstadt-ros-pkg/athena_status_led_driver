#ifndef ATHENA_STATUS_LED_DRIVER_RAINBOW_LOADING_EFFECT_HPP
#define ATHENA_STATUS_LED_DRIVER_RAINBOW_LOADING_EFFECT_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include <cmath>

namespace athena_status_led_driver
{

/**
 * @brief Base effect that shows a smoothly rotating rainbow.
 *
 * Active until the first operating mode message is received, signaling
 * that the node is loaded and waiting for status data.
 */
class RainbowLoadingEffect : public LedEffect
{
public:
  static constexpr double ROTATION_SPEED = 0.3;  // revolutions per second

  explicit RainbowLoadingEffect(size_t led_count)
    : led_count_(led_count) {}

  bool isActive() const override { return active_; }

  void update(double dt) override;
  void render(std::vector<Color>& pixels) override;

  /// Deactivate the rainbow (called when first status is received)
  void deactivate() { active_ = false; }

  /// Re-activate the rainbow
  void activate() { active_ = true; }

  /// Get current offset (for testing)
  double offset() const { return offset_; }

private:
  /// Convert HSV (h in [0,1], s in [0,1], v in [0,1]) to RGB
  static Color hsvToRgb(float h, float s, float v);

  size_t led_count_;
  bool active_ = true;
  double offset_ = 0.0;
};

}  // namespace athena_status_led_driver

#endif  // ATHENA_STATUS_LED_DRIVER_RAINBOW_LOADING_EFFECT_HPP
