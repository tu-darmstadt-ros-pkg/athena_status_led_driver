#include "athena_status_led_driver/effects/power_supply_effect.hpp"

namespace athena_status_led_driver
{

void PowerSupplyEffect::update( double dt )
{
  position_ += dt * ROTATION_SPEED * static_cast<double>( led_count_ );
  if ( position_ >= static_cast<double>( led_count_ ) )
    position_ -= static_cast<double>( led_count_ );
}

void PowerSupplyEffect::render( std::vector<Color> &pixels )
{
  if ( !on_power_supply_ || led_count_ == 0 )
    return;

  Color green( 0, 255, 0 );

  // Space the 4 LEDs evenly around the ring
  double spacing = static_cast<double>( led_count_ ) / NUM_LED_GROUPS;

  for ( int i = 0; i < NUM_LED_GROUPS; ++i ) {
    double led_pos = position_ + i * spacing;
    // Wrap position
    while ( led_pos >= static_cast<double>( led_count_ ) )
      led_pos -= static_cast<double>( led_count_ );
    while ( led_pos < 0.0 ) led_pos += static_cast<double>( led_count_ );

    int start_idx = static_cast<int>( std::floor( led_pos ) ) % static_cast<int>( led_count_ );
    float frac = static_cast<float>( led_pos - std::floor( led_pos ) );

    // The first pixel gets (1 - frac) blending
    pixels[start_idx] = pixels[start_idx].blendOver( green, 1.0f - frac );

    // The middle pixels get full green
    for ( int j = 1; j < NUM_LEDS_PER_GROUP; ++j ) {
      int idx = ( start_idx + j ) % static_cast<int>( led_count_ );
      pixels[idx] = pixels[idx].blendOver( green, 1.0f );
    }

    // The trailing pixel gets frac blending
    int end_idx = ( start_idx + NUM_LEDS_PER_GROUP ) % static_cast<int>( led_count_ );
    pixels[end_idx] = pixels[end_idx].blendOver( green, frac );
  }
}

} // namespace athena_status_led_driver
