#include "athena_status_led_driver/effects/rainbow_loading_effect.hpp"

namespace athena_status_led_driver
{

void RainbowLoadingEffect::update(double dt)
{
  offset_ += dt * ROTATION_SPEED;
  if (offset_ >= 1.0)
    offset_ -= 1.0;
}

void RainbowLoadingEffect::render(std::vector<Color>& pixels)
{
  if (!active_ || led_count_ == 0)
    return;

  for (size_t i = 0; i < pixels.size(); ++i)
  {
    // Map LED position + animation offset to hue [0, 1)
    float hue = static_cast<float>(i) / static_cast<float>(led_count_) +
                static_cast<float>(offset_);
    if (hue >= 1.0f)
      hue -= 1.0f;
    pixels[i] = hsvToRgb(hue, 1.0f, 1.0f);
  }
}

Color RainbowLoadingEffect::hsvToRgb(float h, float s, float v)
{
  float c = v * s;
  float x = c * (1.0f - std::fabs(std::fmod(h * 6.0f, 2.0f) - 1.0f));
  float m = v - c;

  float r = 0, g = 0, b = 0;
  int sector = static_cast<int>(h * 6.0f) % 6;
  switch (sector)
  {
    case 0: r = c; g = x; b = 0; break;
    case 1: r = x; g = c; b = 0; break;
    case 2: r = 0; g = c; b = x; break;
    case 3: r = 0; g = x; b = c; break;
    case 4: r = x; g = 0; b = c; break;
    case 5: r = c; g = 0; b = x; break;
  }

  return Color(
    static_cast<uint8_t>((r + m) * 255.0f),
    static_cast<uint8_t>((g + m) * 255.0f),
    static_cast<uint8_t>((b + m) * 255.0f));
}

}  // namespace athena_status_led_driver
