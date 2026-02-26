#include "athena_status_led_driver/effects/power_supply_effect.hpp"

namespace athena_status_led_driver
{

void PowerSupplyEffect::update(double dt)
{
  position_ += dt * ROTATION_SPEED * static_cast<double>(led_count_);
  if (position_ >= static_cast<double>(led_count_))
    position_ -= static_cast<double>(led_count_);
}

void PowerSupplyEffect::render(std::vector<Color>& pixels)
{
  if (!on_power_supply_ || led_count_ == 0)
    return;

  Color green(0, 255, 0);

  // Space the 4 LEDs evenly around the ring
  double spacing = static_cast<double>(led_count_) / NUM_CHASE_LEDS;

  for (int i = 0; i < NUM_CHASE_LEDS; ++i)
  {
    double led_pos = position_ + i * spacing;
    // Wrap position
    while (led_pos >= static_cast<double>(led_count_))
      led_pos -= static_cast<double>(led_count_);

    // Anti-aliased rendering: blend between the two nearest LEDs
    int idx0 = static_cast<int>(std::floor(led_pos)) % static_cast<int>(led_count_);
    int idx1 = (idx0 + 1) % static_cast<int>(led_count_);
    float frac = static_cast<float>(led_pos - std::floor(led_pos));

    pixels[idx0] = pixels[idx0].blendOver(green, 1.0f - frac);
    pixels[idx1] = pixels[idx1].blendOver(green, frac);
  }
}

}  // namespace athena_status_led_driver
