#include "athena_status_led_driver/effects/battery_fault_effect.hpp"
#include <algorithm>
#include <cmath>

namespace athena_status_led_driver
{

BatteryFaultEffect::BatteryFaultEffect( size_t led_count ) : led_count_( led_count ) { }

void BatteryFaultEffect::update( double dt )
{
  toggle_timer_ += dt;
  while ( toggle_timer_ >= TOGGLE_PERIOD ) {
    toggle_timer_ -= TOGGLE_PERIOD;
    inverted_ = !inverted_;
  }
}

void BatteryFaultEffect::render( std::vector<Color> &pixels )
{
  if ( led_count_ == 0 )
    return;

  // Battery 1 is centered at +PI/2, Battery 2 at -PI/2, matching BatteryConnectionEffect.
  if ( faulty_[0] )
    renderSide( pixels, static_cast<double>( led_count_ ) * 0.25 );
  if ( faulty_[1] )
    renderSide( pixels, static_cast<double>( led_count_ ) * 0.75 );
}

void BatteryFaultEffect::renderSide( std::vector<Color> &pixels, double center_idx ) const
{
  Color red = Color( 255, 0, 0 ).scaled( BRIGHTNESS );
  double half_width = static_cast<double>( led_count_ ) / 4.0;

  for ( size_t j = 0; j < led_count_; ++j ) {
    // Signed distance from the side center, wrapped to [-led_count/2, led_count/2).
    double diff = static_cast<double>( j ) - center_idx;
    while ( diff < -static_cast<double>( led_count_ ) / 2.0 )
      diff += static_cast<double>( led_count_ );
    while ( diff >= static_cast<double>( led_count_ ) / 2.0 )
      diff -= static_cast<double>( led_count_ );

    if ( std::abs( diff ) > half_width )
      continue;

    // Tile the arc into GROUP_SIZE-wide segments; even segments are X (red).
    int group = static_cast<int>( std::floor( ( diff + half_width ) / GROUP_SIZE ) );
    bool is_red = ( group % 2 == 0 ) != inverted_;
    if ( is_red )
      pixels[j] = pixels[j].blendOver( red, 1.0f );
  }
}

void BatteryFaultEffect::updateBatteryState(
    const std::array<uint16_t, CELLS_PER_BATTERY> &cell_voltages_battery1,
    const std::array<uint16_t, CELLS_PER_BATTERY> &cell_voltages_battery2 )
{
  faulty_[0] = isImproperlyConnected( cell_voltages_battery1 );
  faulty_[1] = isImproperlyConnected( cell_voltages_battery2 );
}

bool BatteryFaultEffect::isImproperlyConnected( const std::array<uint16_t, CELLS_PER_BATTERY> &cells )
{
  bool has_bad_cell =
      std::any_of( cells.begin(), cells.end(), []( uint16_t v ) { return v < BAD_CELL_MV; } );
  bool present =
      std::any_of( cells.begin(), cells.end(), []( uint16_t v ) { return v >= MIN_PRESENT_MV; } );
  return has_bad_cell && present;
}

} // namespace athena_status_led_driver
