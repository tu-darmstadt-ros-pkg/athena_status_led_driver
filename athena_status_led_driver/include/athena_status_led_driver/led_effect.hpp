#ifndef ATHENA_STATUS_LED_DRIVER_LED_EFFECT_HPP
#define ATHENA_STATUS_LED_DRIVER_LED_EFFECT_HPP

#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>

namespace athena_status_led_driver
{

struct Color
{
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;

  Color() = default;
  Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}

  bool operator==(const Color& o) const { return r == o.r && g == o.g && b == o.b; }
  bool operator!=(const Color& o) const { return !(*this == o); }

  /// Scale color by a factor [0.0, 1.0]
  Color scaled(float factor) const
  {
    factor = std::clamp(factor, 0.0f, 1.0f);
    return Color(
      static_cast<uint8_t>(r * factor),
      static_cast<uint8_t>(g * factor),
      static_cast<uint8_t>(b * factor));
  }

  /// Blend another color on top with given alpha [0.0, 1.0]
  Color blendOver(const Color& overlay, float alpha) const
  {
    alpha = std::clamp(alpha, 0.0f, 1.0f);
    float inv = 1.0f - alpha;
    return Color(
      static_cast<uint8_t>(std::min(255.0f, r * inv + overlay.r * alpha)),
      static_cast<uint8_t>(std::min(255.0f, g * inv + overlay.g * alpha)),
      static_cast<uint8_t>(std::min(255.0f, b * inv + overlay.b * alpha)));
  }
};

/**
 * @brief Abstract base class for LED effects.
 *
 * Effects are rendered in priority order. "Fill" effects (like operating mode)
 * write all pixels. "Blend" effects (like battery pulse) selectively modify
 * existing pixels. The distinction is purely in each effect's render()
 * implementation — no classification flag needed.
 */
class LedEffect
{
public:
  virtual ~LedEffect() = default;

  /// Whether the effect should currently be rendered
  virtual bool isActive() const = 0;

  /// Advance animation state by dt seconds
  virtual void update(double dt) = 0;

  /// Render into or blend onto the pixel buffer
  virtual void render(std::vector<Color>& pixels) = 0;
};

}  // namespace athena_status_led_driver

#endif  // ATHENA_STATUS_LED_DRIVER_LED_EFFECT_HPP
