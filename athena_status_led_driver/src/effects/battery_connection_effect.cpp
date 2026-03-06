#include "athena_status_led_driver/effects/battery_connection_effect.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace athena_status_led_driver
{

BatteryConnectionEffect::BatteryConnectionEffect( size_t led_count ) : led_count_( led_count ) { }

bool BatteryConnectionEffect::isActive() const
{
  return std::any_of( batteries_.begin(), batteries_.end(),
                      []( const BatteryState &b ) { return b.state != State::INACTIVE; } );
}

void BatteryConnectionEffect::update( double dt )
{
  for ( auto &b : batteries_ ) {
    switch ( b.state ) {
    case State::CONNECTING:
      b.progress += dt / CONN_DURATION;
      if ( b.progress >= 1.0 ) {
        b.progress = 1.0;
        b.state = State::INACTIVE;
      }
      break;
    case State::DISCONNECTING_QUICK_FILL:
      b.progress += dt / DISCONN_QUICK_FILL_DURATION;
      if ( b.progress >= 1.0 ) {
        b.progress = 1.0;
        b.state = State::DISCONNECTING_SHRINK;
      }
      break;
    case State::DISCONNECTING_SHRINK:
      b.progress -= dt / DISCONN_SHRINK_DURATION;
      if ( b.progress <= 0.0 ) {
        b.progress = 0.0;
        b.state = State::INACTIVE;
      }
      break;
    default:
      break;
    }
  }
}

void BatteryConnectionEffect::render( std::vector<Color> &pixels )
{
  if ( led_count_ == 0 )
    return;

  Color green = Color( 0, 255, 0 ).scaled( BRIGHTNESS );
  double led_per_rad = static_cast<double>( led_count_ ) / ( 2.0 * M_PI );

  for ( size_t i = 0; i < 2; ++i ) {
    const auto &b = batteries_[i];
    if ( b.state == State::INACTIVE )
      continue;

    // Battery 1 is at +PI/2, Battery 2 is at -PI/2 in robot frame
    double battery_angle_ring = ( i == 0 ) ? ( M_PI / 2.0 ) : ( -M_PI / 2.0 );

    // Normalize to [0, 2PI)
    while ( battery_angle_ring < 0 ) battery_angle_ring += 2.0 * M_PI;
    while ( battery_angle_ring >= 2.0 * M_PI ) battery_angle_ring -= 2.0 * M_PI;

    double center_idx = battery_angle_ring * led_per_rad;

    // Each battery covers half the ring (55 LEDs for 110 count)
    double half_width = static_cast<double>( led_count_ ) / 4.0;
    double current_half_width = half_width * b.progress;

    for ( size_t j = 0; j < led_count_; ++j ) {
      double dist = std::abs( static_cast<double>( j ) - center_idx );
      if ( dist > static_cast<double>( led_count_ ) / 2.0 ) {
        dist = static_cast<double>( led_count_ ) - dist;
      }

      if ( dist <= current_half_width ) {
        pixels[j] = green;
      }
    }
  }
}

void BatteryConnectionEffect::updateBatteryState( const std::array<uint16_t, CELLS_PER_BATTERY> &cells1,
                                                  const std::array<uint16_t, CELLS_PER_BATTERY> &cells2 )
{
  bool conn1 = isConnected( cells1 );
  bool conn2 = isConnected( cells2 );

  std::array<bool, 2> new_conns = { conn1, conn2 };

  for ( size_t i = 0; i < 2; ++i ) {
    if ( new_conns[i] && !batteries_[i].connected ) {
      // Trigger connection animation
      RCLCPP_INFO( rclcpp::get_logger( "athena_status_led_driver" ), "Battery %zu connected", i + 1 );
      batteries_[i].state = State::CONNECTING;
      batteries_[i].progress = 0.0;
    } else if ( !new_conns[i] && batteries_[i].connected ) {
      // Trigger disconnection animation
      RCLCPP_INFO( rclcpp::get_logger( "athena_status_led_driver" ), "Battery %zu disconnected",
                   i + 1 );
      batteries_[i].state = State::DISCONNECTING_QUICK_FILL;
      batteries_[i].progress = 0.0;
    }
    batteries_[i].connected = new_conns[i];
  }
}

bool BatteryConnectionEffect::isConnected( const std::array<uint16_t, CELLS_PER_BATTERY> &cells )
{
  return std::count_if( cells.begin(), cells.end(), []( uint16_t v ) {
           return v >= 2800 && v <= 4500;
         } ) >= static_cast<int>( cells.size() ) / 2;
}

} // namespace athena_status_led_driver
