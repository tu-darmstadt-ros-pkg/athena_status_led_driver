#include "athena_status_led_driver/led_ring_controller.hpp"
#include <algorithm>

namespace athena_status_led_driver
{

LedRingController::LedRingController( size_t led_count, std::shared_ptr<LedTransport> transport )
    : led_count_( led_count ), transport_( std::move( transport ) ), pixels_( led_count )
{
}

void LedRingController::addEffect( std::shared_ptr<LedEffect> effect )
{
  effects_.push_back( std::move( effect ) );
}

void LedRingController::setBrightness( float brightness )
{
  brightness_ = std::clamp( brightness, 0.0f, 1.0f );
}

float LedRingController::brightness() const { return brightness_; }

void LedRingController::tick( double dt )
{
  // Update all effects
  for ( auto &effect : effects_ ) effect->update( dt );

  // Clear pixels
  std::fill( pixels_.begin(), pixels_.end(), Color( 0, 0, 0 ) );

  // Render all active effects in priority order
  for ( auto &effect : effects_ ) {
    if ( effect->isActive() )
      effect->render( pixels_ );
  }

  // Apply global brightness
  if ( brightness_ < 1.0f ) {
    for ( auto &pixel : pixels_ ) pixel = pixel.scaled( brightness_ );
  }

  // Send frame
  if ( transport_ )
    transport_->send( pixels_ );
}

const std::vector<Color> &LedRingController::pixels() const { return pixels_; }

std::shared_ptr<LedTransport> LedRingController::transport() const { return transport_; }

} // namespace athena_status_led_driver
