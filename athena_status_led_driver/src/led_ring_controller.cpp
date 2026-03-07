#include "athena_status_led_driver/led_ring_controller.hpp"
#include <algorithm>
#include <cmath>

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

void LedRingController::setRotation( double angle_rad ) { rotation_rad_ = angle_rad; }

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

  // Apply rotation
  if ( std::abs( rotation_rad_ ) > 1e-6 && led_count_ > 0 ) {
    double led_per_rad = static_cast<double>( led_count_ ) / ( 2.0 * M_PI );
    // Note: If arm moves CCW (+), the LED content must move CW (-) to stay fixed in base frame.
    // However, the LED indices typically follow CCW order on the ring.
    // So index_shift = round(-rotation_rad * led_per_rad)
    int shift = static_cast<int>( std::round( -rotation_rad_ * led_per_rad ) );

    // Normalize shift to [0, led_count_)
    shift = ( shift % static_cast<int>( led_count_ ) + static_cast<int>( led_count_ ) ) %
            static_cast<int>( led_count_ );

    if ( shift != 0 ) {
      // std::rotate(first, middle, last) moves middle to first.
      // We want to shift elements right by 'shift', which is equivalent to moving
      // the element at 'size - shift' to the beginning.
      std::rotate( pixels_.begin(), pixels_.begin() + ( led_count_ - shift ), pixels_.end() );
    }
  }

  // Send frame
  if ( transport_ )
    transport_->send( pixels_ );
}

const std::vector<Color> &LedRingController::pixels() const { return pixels_; }

std::shared_ptr<LedTransport> LedRingController::transport() const { return transport_; }

} // namespace athena_status_led_driver
