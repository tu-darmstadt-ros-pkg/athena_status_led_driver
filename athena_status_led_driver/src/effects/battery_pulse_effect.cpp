#include "athena_status_led_driver/effects/battery_pulse_effect.hpp"

namespace athena_status_led_driver
{

void BatteryPulseEffect::update( double dt )
{
  low_duration_ += dt;
  if ( low_duration_ < 10.0 ) {
    phase_ = 0.0;
    return;
  }
  phase_ += dt * PULSE_FREQUENCY_HZ * 2.0 * M_PI;
  if ( phase_ > 2.0 * M_PI )
    phase_ -= 2.0 * M_PI;
}

void BatteryPulseEffect::render( std::vector<Color> &pixels )
{
  if ( low_duration_ < 10.0 )
    return;

  // Only positive part of sine wave so we have half the time no pulse
  double pulse = std::max( 0.0, std::sin( phase_ ) );
  // Scale the pulse intensity to 1.2 and cap at 1.0 to remain at full brightness for a moment
  float alpha = std::min( 1.0f, static_cast<float>( pulse ) * 1.2f );

  Color red = Color( 255, 0, 0 ).scaled( BRIGHTNESS );
  for ( auto &pixel : pixels ) pixel = pixel.blendOver( red, alpha );
}

void BatteryPulseEffect::updateBatteryState(
    const std::array<uint16_t, CELLS_PER_BATTERY> &cell_voltages_battery1,
    const std::array<uint16_t, CELLS_PER_BATTERY> &cell_voltages_battery2, uint16_t low_cell_mv )
{
  bool bat1_connected = isConnected( cell_voltages_battery1 );
  bool bat2_connected = isConnected( cell_voltages_battery2 );

  // Hysteresis: if already in low_battery state, we need a higher voltage to clear it.
  uint16_t threshold = low_battery_ ? ( low_cell_mv + HYSTERESIS_MV ) : low_cell_mv;

  bool bat1_low = hasLowCell( cell_voltages_battery1, threshold );
  bool bat2_low = hasLowCell( cell_voltages_battery2, threshold );

  if ( bat1_connected && bat2_connected ) {
    low_battery_ = bat1_low && bat2_low;
  } else if ( !bat1_connected && !bat2_connected ) {
    low_battery_ = false;
  } else if ( !bat2_connected ) {
    low_battery_ = bat1_low;
  } else {
    low_battery_ = bat2_low;
  }
  if ( !low_battery_ )
    low_duration_ = 0;
}

bool BatteryPulseEffect::isConnected( const std::array<uint16_t, CELLS_PER_BATTERY> &cells )
{
  return std::count_if( cells.begin(), cells.end(), []( uint16_t v ) {
           return v >= 2800 && v <= 4500;
         } ) >= static_cast<int>( cells.size() / 2 );
}

bool BatteryPulseEffect::hasLowCell( const std::array<uint16_t, CELLS_PER_BATTERY> &cells,
                                     uint16_t threshold_mv )
{
  return std::any_of( cells.begin(), cells.end(),
                      [threshold_mv]( uint16_t v ) { return v < threshold_mv; } );
}

} // namespace athena_status_led_driver
