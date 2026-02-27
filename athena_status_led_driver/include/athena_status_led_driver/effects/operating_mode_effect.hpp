#ifndef ATHENA_STATUS_LED_DRIVER_OPERATING_MODE_EFFECT_HPP
#define ATHENA_STATUS_LED_DRIVER_OPERATING_MODE_EFFECT_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include <string>
#include <unordered_map>

namespace athena_status_led_driver
{

/**
 * @brief Base effect that fills all LEDs with the current operating mode color.
 *
 * Mode colors:
 *   autonomous   → blue (0, 80, 255)
 *   driving      → yellowish orange (255, 180, 0)
 *   manipulation → reddish orange (255, 60, 0)
 *   safe         → green (0, 255, 0)
 */
class OperatingModeEffect : public LedEffect
{
public:
  OperatingModeEffect();

  bool isActive() const override { return has_mode_; }

  void update(double /*dt*/) override {}

  void render(std::vector<Color>& pixels) override;

  /// Set the current operating mode. Unknown modes are ignored.
  void setMode(const std::string& mode);

  /// Get the current color (for testing)
  Color currentColor() const { return current_color_; }

  /// Check if a mode string is recognized
  bool isModeValid(const std::string& mode) const
  {
    return mode_colors_.count(mode) > 0;
  }

  /// Get the currently active mode string
  std::string currentMode() const { return current_mode_; }

private:
  bool has_mode_ = false;
  std::string current_mode_;
  Color current_color_;
  std::unordered_map<std::string, Color> mode_colors_;
};

}  // namespace athena_status_led_driver

#endif  // ATHENA_STATUS_LED_DRIVER_OPERATING_MODE_EFFECT_HPP
