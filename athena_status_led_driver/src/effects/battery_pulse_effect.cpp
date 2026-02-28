#include "athena_status_led_driver/effects/battery_pulse_effect.hpp"

namespace athena_status_led_driver
{

void BatteryPulseEffect::update( double dt )
{
  phase_ += dt * PULSE_FREQUENCY_HZ * 2.0 * M_PI;
  if ( phase_ > 2.0 * M_PI )
    phase_ -= 2.0 * M_PI;
}

void BatteryPulseEffect::render( std::vector<Color> &pixels )
{
  if ( !low_battery_ )
    return;

  // Only positive part of sine wave so we have half the time no pulse
  double pulse = std::max( 0.0, std::sin( phase_ ) );
  // Scale the pulse intensity (0.0 to 0.8 to avoid full washout)
  float alpha = static_cast<float>( pulse ) * 0.8f;

  Color red( 255, 0, 0 );
  for ( auto &pixel : pixels ) pixel = pixel.blendOver( red, alpha );
}

void BatteryPulseEffect::updateBatteryState(
    const std::array<uint16_t, CELLS_PER_BATTERY> &cell_voltages_battery1,
    const std::array<uint16_t, CELLS_PER_BATTERY> &cell_voltages_battery2, uint16_t low_cell_mv )
{
  bool bat1_connected = isConnected( cell_voltages_battery1 );
  bool bat2_connected = isConnected( cell_voltages_battery2 );
  bool bat1_low = hasLowCell( cell_voltages_battery1, low_cell_mv );
  bool bat2_low = hasLowCell( cell_voltages_battery2, low_cell_mv );
  if ( bat1_connected && bat2_connected ) {
    low_battery_ = bat1_low && bat2_low;
  } else if ( !bat1_connected ) {
    low_battery_ = bat2_low;
  } else if ( !bat2_connected ) {
    low_battery_ = bat1_low;
  } else {
    low_battery_ = false;
  }
}

bool BatteryPulseEffect::isConnected( const std::array<uint16_t, CELLS_PER_BATTERY> &cells )
{
  return std::any_of( cells.begin(), cells.end(), []( uint16_t v ) { return v > 0; } );
}

bool BatteryPulseEffect::hasLowCell( const std::array<uint16_t, CELLS_PER_BATTERY> &cells,
                                     uint16_t threshold_mv )
{
  return std::any_of( cells.begin(), cells.end(),
                      [threshold_mv]( uint16_t v ) { return v < threshold_mv; } );
}

} // namespace athena_status_led_driver
