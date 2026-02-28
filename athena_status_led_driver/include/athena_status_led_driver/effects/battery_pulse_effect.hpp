#ifndef ATHENA_STATUS_LED_DRIVER_BATTERY_PULSE_EFFECT_HPP
#define ATHENA_STATUS_LED_DRIVER_BATTERY_PULSE_EFFECT_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include <array>
#include <cmath>

namespace athena_status_led_driver
{

/**
 * @brief Overlay effect that pulses red when both batteries have a cell below threshold.
 *
 * The pulse is a smooth sine wave affecting all LEDs.
 * Designed to be easily extended to per-battery percentage display.
 */
class BatteryPulseEffect : public LedEffect
{
public:
  static constexpr uint16_t DEFAULT_LOW_CELL_MV = 3800;  // 3.8V
  static constexpr size_t CELLS_PER_BATTERY = 8;
  static constexpr double PULSE_FREQUENCY_HZ = 0.25;  // pulses per second

  bool isActive() const override { return low_battery_; }

  void update(double dt) override;
  void render(std::vector<Color>& pixels) override;

  /**
   * @brief Update battery state from a BatteryStatus message.
   *
   * @param cell_voltages_battery1  per-cell voltages in mV (8 cells)
   * @param cell_voltages_battery2  per-cell voltages in mV (8 cells)
   * @param low_cell_mv             threshold in mV (default 3800 = 3.8V)
   *
   * Low battery triggers when BOTH batteries have at least one cell below threshold.
   */
  void updateBatteryState(
    const std::array<uint16_t, CELLS_PER_BATTERY>& cell_voltages_battery1,
    const std::array<uint16_t, CELLS_PER_BATTERY>& cell_voltages_battery2,
    uint16_t low_cell_mv = DEFAULT_LOW_CELL_MV);

  /// Get current low-battery state (for testing)
  bool isLowBattery() const { return low_battery_; }

  /// Get current pulse phase (for testing)
  double phase() const { return phase_; }

private:
  static bool isConnected(
    const std::array<uint16_t, CELLS_PER_BATTERY>& cells);
  static bool hasLowCell(
    const std::array<uint16_t, CELLS_PER_BATTERY>& cells,
    uint16_t threshold_mv);

  bool low_battery_ = false;
  double phase_ = 0.0;
};

}  // namespace athena_status_led_driver

#endif  // ATHENA_STATUS_LED_DRIVER_BATTERY_PULSE_EFFECT_HPP
