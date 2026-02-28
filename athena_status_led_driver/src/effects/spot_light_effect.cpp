#include "athena_status_led_driver/effects/spot_light_effect.hpp"

namespace athena_status_led_driver
{

void SpotLightEffect::render( std::vector<Color> &pixels )
{
  if ( !enabled_ || led_count_ == 0 )
    return;

  Color white( 255, 255, 255 );
  float scaled_brightness = brightness_;

  for ( size_t i = 0; i < pixels.size(); ++i ) {
    float alpha = ledAlpha( i );
    if ( alpha > 0.0f )
      pixels[i] = pixels[i].blendOver( white, alpha * scaled_brightness );
  }
}

float SpotLightEffect::ledAlpha( size_t led_index ) const
{
  if ( width_deg_ >= 360.0f )
    return 1.0f;

  // Degrees per LED
  float deg_per_led = 360.0f / static_cast<float>( led_count_ );
  float led_angle = static_cast<float>( led_index ) * deg_per_led;

  // Angular distance from direction center (smallest arc)
  float diff = led_angle - direction_deg_;
  // Normalize to [-180, 180]
  diff = std::fmod( diff + 540.0f, 360.0f ) - 180.0f;

  float half_width = width_deg_ / 2.0f;
  float abs_diff = std::fabs( diff );

  if ( abs_diff <= half_width )
    return 1.0f;

  // Smooth falloff over one LED width at the edge
  float edge = half_width + deg_per_led;
  if ( abs_diff < edge )
    return 1.0f - ( abs_diff - half_width ) / deg_per_led;

  return 0.0f;
}

} // namespace athena_status_led_driver
