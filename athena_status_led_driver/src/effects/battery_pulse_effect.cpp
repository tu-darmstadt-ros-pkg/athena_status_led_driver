#include "athena_status_led_driver/effects/battery_pulse_effect.hpp"

namespace athena_status_led_driver
{

void BatteryPulseEffect::update(double dt)
{
  phase_ += dt * PULSE_FREQUENCY_HZ * 2.0 * M_PI;
  if (phase_ > 2.0 * M_PI)
    phase_ -= 2.0 * M_PI;
}

void BatteryPulseEffect::render(std::vector<Color>& pixels)
{
  if (!low_battery_)
    return;

  // Sine pulse: 0.0 to 1.0 (half-wave, only positive)
  float pulse = static_cast<float>((std::sin(phase_) + 1.0) * 0.5);
  // Scale the pulse intensity (0.0 to 0.8 to avoid full washout)
  float alpha = pulse * 0.8f;

  Color red(255, 0, 0);
  for (auto& pixel : pixels)
    pixel = pixel.blendOver(red, alpha);
}

void BatteryPulseEffect::updateBatteryState(
  const std::array<uint16_t, CELLS_PER_BATTERY>& cell_voltages_battery1,
  const std::array<uint16_t, CELLS_PER_BATTERY>& cell_voltages_battery2,
  uint16_t low_cell_mv)
{
  bool bat1_low = hasLowCell(cell_voltages_battery1, low_cell_mv);
  bool bat2_low = hasLowCell(cell_voltages_battery2, low_cell_mv);
  low_battery_ = bat1_low && bat2_low;
}

bool BatteryPulseEffect::hasLowCell(
  const std::array<uint16_t, CELLS_PER_BATTERY>& cells,
  uint16_t threshold_mv)
{
  for (auto v : cells)
  {
    // Skip cells that report 0 (disconnected/unused)
    if (v > 0 && v < threshold_mv)
      return true;
  }
  return false;
}

}  // namespace athena_status_led_driver
