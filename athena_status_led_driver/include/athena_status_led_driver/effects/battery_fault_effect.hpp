#ifndef ATHENA_STATUS_LED_DRIVER_BATTERY_FAULT_EFFECT_HPP
#define ATHENA_STATUS_LED_DRIVER_BATTERY_FAULT_EFFECT_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include <array>
#include <cstddef>
#include <vector>

namespace athena_status_led_driver
{

/**
 * @brief Overlay effect that warns when a battery is not properly connected.
 *
 * A battery counts as improperly connected when at least one cell reads below
 * BAD_CELL_MV while the pack is still partially present (at least one cell reads
 * above MIN_PRESENT_MV). A fully disconnected pack (all cells ~0) is left to the
 * BatteryConnectionEffect and does not trigger this warning.
 *
 * The affected battery's half of the ring shows an alternating red/off dash
 * pattern (X = red, O = untouched base). The X and O segments swap every
 * TOGGLE_PERIOD seconds so the side blinks. Each battery is handled
 * independently on its own side.
 */
class BatteryFaultEffect : public LedEffect
{
public:
  static constexpr size_t CELLS_PER_BATTERY = 8;
  static constexpr uint16_t BAD_CELL_MV = 3000;    // 3.0V: cell below this is a bad reading
  static constexpr uint16_t MIN_PRESENT_MV = 3000; // pack present if any cell reads above this
  static constexpr size_t GROUP_SIZE = 3;          // LEDs per X or O segment
  static constexpr double TOGGLE_PERIOD = 1.0;     // seconds between X/O swaps
  static constexpr float BRIGHTNESS = 0.8f;

  explicit BatteryFaultEffect( size_t led_count );

  bool isActive() const override { return faulty_[0] || faulty_[1]; }

  void update( double dt ) override;

  void render( std::vector<Color> &pixels ) override;

  /**
   * @brief Update battery state from the per-cell voltages of both batteries.
   *
   * @param cell_voltages_battery1  per-cell voltages in mV (8 cells)
   * @param cell_voltages_battery2  per-cell voltages in mV (8 cells)
   */
  void updateBatteryState( const std::array<uint16_t, CELLS_PER_BATTERY> &cell_voltages_battery1,
                           const std::array<uint16_t, CELLS_PER_BATTERY> &cell_voltages_battery2 );

  /// Whether the given battery (0 or 1) is currently flagged as faulty (for testing)
  bool isFaulty( size_t battery ) const { return faulty_[battery]; }

  /// Whether the X/O pattern is currently inverted (for testing)
  bool inverted() const { return inverted_; }

private:
  static bool isImproperlyConnected( const std::array<uint16_t, CELLS_PER_BATTERY> &cells );

  void renderSide( std::vector<Color> &pixels, double center_idx ) const;

  size_t led_count_;
  std::array<bool, 2> faulty_ = { false, false };
  double toggle_timer_ = 0.0;
  bool inverted_ = false;
};

} // namespace athena_status_led_driver

#endif // ATHENA_STATUS_LED_DRIVER_BATTERY_FAULT_EFFECT_HPP
