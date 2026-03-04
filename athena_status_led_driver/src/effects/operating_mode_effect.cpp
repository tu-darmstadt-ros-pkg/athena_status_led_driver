#include "athena_status_led_driver/effects/operating_mode_effect.hpp"

namespace athena_status_led_driver
{

OperatingModeEffect::OperatingModeEffect()
{
  mode_colors_["autonomous"] = Color( 0, 80, 255 );
  mode_colors_["driving"] = Color( 255, 180, 0 );
  mode_colors_["manipulation"] = Color( 255, 60, 0 );
  mode_colors_["safe"] = Color( 0, 255, 0 );
}

void OperatingModeEffect::render( std::vector<Color> &pixels )
{
  if ( !has_mode_ )
    return;
  std::fill( pixels.begin(), pixels.end(), current_color_.scaled( BRIGHTNESS ) );
}

void OperatingModeEffect::setMode( const std::string &mode )
{
  auto it = mode_colors_.find( mode );
  if ( it != mode_colors_.end() ) {
    current_color_ = it->second;
    has_mode_ = true;
  }
}

} // namespace athena_status_led_driver
