#ifndef ATHENA_STATUS_LED_DRIVER_BATTERY_CONNECTION_EFFECT_HPP
#define ATHENA_STATUS_LED_DRIVER_BATTERY_CONNECTION_EFFECT_HPP

#include "athena_status_led_driver/led_effect.hpp"
#include <array>
#include <vector>

namespace athena_status_led_driver
{

/**
 * @brief Effect that visualizes battery connection and disconnection.
 *
 * When a battery is connected, it fills from the center of its side.
 * When disconnected, it shrinks back to the center.
 * The orientation is adjusted based on a joint angle.
 */
class BatteryConnectionEffect : public LedEffect
{
public:
  static constexpr double BRIGHTNESS = 0.5;
  static constexpr size_t CELLS_PER_BATTERY = 8;
  static constexpr double CONN_DURATION = 1.0;
  static constexpr double DISCONN_QUICK_FILL_DURATION = 0.2;
  static constexpr double DISCONN_SHRINK_DURATION = 0.8;

  explicit BatteryConnectionEffect( size_t led_count );

  bool isActive() const override;

  void update( double dt ) override;
  void render( std::vector<Color> &pixels ) override;

  /**
   * @brief Update battery state and trigger animations if needed.
   */
  void updateBatteryState( const std::array<uint16_t, CELLS_PER_BATTERY> &cells1,
                           const std::array<uint16_t, CELLS_PER_BATTERY> &cells2 );

private:
  enum class State { INACTIVE, CONNECTING, DISCONNECTING_QUICK_FILL, DISCONNECTING_SHRINK };

  struct BatteryState {
    bool connected = false;
    State state = State::INACTIVE;
    double progress = 0.0;
  };

  static bool isConnected( const std::array<uint16_t, CELLS_PER_BATTERY> &cells );

  size_t led_count_;
  std::array<BatteryState, 2> batteries_;
};

} // namespace athena_status_led_driver

#endif // ATHENA_STATUS_LED_DRIVER_BATTERY_CONNECTION_EFFECT_HPP
