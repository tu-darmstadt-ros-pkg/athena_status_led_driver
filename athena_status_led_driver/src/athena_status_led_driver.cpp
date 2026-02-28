#include "athena_status_led_driver/athena_status_led_driver.hpp"
#include "athena_status_led_driver/effects/battery_pulse_effect.hpp"
#include "athena_status_led_driver/effects/operating_mode_effect.hpp"
#include "athena_status_led_driver/effects/power_supply_effect.hpp"
#include "athena_status_led_driver/effects/rainbow_loading_effect.hpp"
#include "athena_status_led_driver/effects/spot_light_effect.hpp"
#include "athena_status_led_driver/spi_transport.hpp"
#include "athena_status_led_driver/terminal_transport.hpp"

#include <algorithm>
#include <functional>

namespace athena_status_led_driver
{

AthenaStatusLedDriver::AthenaStatusLedDriver() : Node( "athena_status_led_driver" )
{
  setup();

  // Validate update rate — fall back to 30 Hz if invalid
  if ( update_rate_hz_ <= 0.0 ) {
    RCLCPP_WARN( get_logger(), "Invalid update_rate_hz (%.1f), falling back to 30.0 Hz",
                 update_rate_hz_ );
    update_rate_hz_ = 30.0;
  }

  last_tick_time_ = now();
  auto period = std::chrono::duration<double>( 1.0 / update_rate_hz_ );
  timer_ = create_wall_timer( std::chrono::duration_cast<std::chrono::nanoseconds>( period ),
                              std::bind( &AthenaStatusLedDriver::onTimer, this ) );
}

AthenaStatusLedDriver::~AthenaStatusLedDriver()
{
  // cleanUp() closes the transport, which already turns off all LEDs
  cleanUp();
}

void AthenaStatusLedDriver::setup()
{
  // Declare parameters
  declare_readonly_parameter( "led_count", led_count_, "Number of LEDs on the ring" );
  declare_readonly_parameter( "spi_device", spi_device_, "SPI device path" );
  declare_readonly_parameter( "spi_speed_hz", spi_speed_hz_, "SPI clock frequency in Hz" );
  declare_readonly_parameter( "update_rate_hz", update_rate_hz_, "LED update rate in Hz" );
  declare_readonly_parameter( "simulate", simulate_,
                              "Use terminal visualization instead of SPI hardware" );
  declare_reconfigurable_parameter(
      "global_brightness", std::ref( global_brightness_ ), "Global brightness (0.0 - 1.0)",
      hector::ParameterOptions<double>().setRange( 0.0, 1.0, 0.0 ).onUpdate( [this]( const double &value ) {
        if ( controller_ )
          controller_->setBrightness( static_cast<float>( value ) );
      } ) );

  // Create transport — terminal visualization when simulating, SPI hardware otherwise
  if ( simulate_ ) {
    RCLCPP_INFO( get_logger(), "Simulation mode: rendering LED ring in terminal" );
    transport_ = std::make_shared<TerminalTransport>();
  } else {
    transport_ = std::make_shared<SpiTransport>( spi_device_, static_cast<uint32_t>( spi_speed_hz_ ),
                                                 static_cast<uint32_t>( led_count_ ) );
  }

  if ( !transport_->open() ) {
    if ( simulate_ ) {
      RCLCPP_ERROR( get_logger(), "Failed to open terminal transport" );
    } else {
      auto spi = std::dynamic_pointer_cast<SpiTransport>( transport_ );
      RCLCPP_ERROR( get_logger(), "Failed to open SPI transport at %s: %s", spi_device_.c_str(),
                    spi ? spi->lastError().c_str() : "unknown" );
    }
  }

  // Create controller
  controller_ = std::make_unique<LedRingController>( static_cast<size_t>( led_count_ ), transport_ );
  controller_->setBrightness( static_cast<float>( global_brightness_ ) );

  // Create effects
  operating_mode_effect_ = std::make_shared<OperatingModeEffect>();
  battery_pulse_effect_ = std::make_shared<BatteryPulseEffect>();
  power_supply_effect_ = std::make_shared<PowerSupplyEffect>( static_cast<size_t>( led_count_ ) );
  spot_light_effect_ = std::make_shared<SpotLightEffect>( static_cast<size_t>( led_count_ ) );
  rainbow_loading_effect_ =
      std::make_shared<RainbowLoadingEffect>( static_cast<size_t>( led_count_ ) );

  // Wire effects in render order (later effects render on top of earlier ones):
  controller_->addEffect( rainbow_loading_effect_ );
  controller_->addEffect( operating_mode_effect_ );
  controller_->addEffect( battery_pulse_effect_ );
  controller_->addEffect( power_supply_effect_ );
  controller_->addEffect( spot_light_effect_ );

  // Create subscribers
  operating_mode_sub_ = create_subscription<std_msgs::msg::String>(
      "operating_mode", rclcpp::QoS( 1 ).transient_local(),
      std::bind( &AthenaStatusLedDriver::onOperatingMode, this, std::placeholders::_1 ) );

  battery_sub_ = create_subscription<athena_firmware_interface_msgs::msg::BatteryStatus>(
      "battery", rclcpp::SensorDataQoS(),
      std::bind( &AthenaStatusLedDriver::onBatteryStatus, this, std::placeholders::_1 ) );

  // Create service
  set_spot_light_srv_ = create_service<athena_status_led_driver_msgs::srv::SetSpotLight>(
      "~/set_spot_light", std::bind( &AthenaStatusLedDriver::onSetSpotLight, this,
                                     std::placeholders::_1, std::placeholders::_2 ) );
}

void AthenaStatusLedDriver::cleanUp()
{
  timer_.reset();
  operating_mode_sub_.reset();
  battery_sub_.reset();
  set_spot_light_srv_.reset();

  controller_.reset();

  operating_mode_effect_.reset();
  battery_pulse_effect_.reset();
  power_supply_effect_.reset();
  spot_light_effect_.reset();
  rainbow_loading_effect_.reset();

  if ( transport_ ) {
    transport_->close();
    transport_.reset();
  }
}

void AthenaStatusLedDriver::onTimer()
{
  auto current_time = now();
  double dt = ( current_time - last_tick_time_ ).seconds();
  last_tick_time_ = current_time;

  // Clamp dt to avoid jumps on startup or clock resets
  dt = std::clamp( dt, 0.0, 0.5 );

  controller_->tick( dt );
}

void AthenaStatusLedDriver::onOperatingMode( const std_msgs::msg::String::SharedPtr msg )
{
  if ( !operating_mode_effect_->isModeValid( msg->data ) ) {
    RCLCPP_WARN( get_logger(), "Unknown operating mode: '%s'", msg->data.c_str() );
    return;
  }
  RCLCPP_DEBUG( get_logger(), "Operating mode: %s", msg->data.c_str() );
  operating_mode_effect_->setMode( msg->data );

  // Deactivate rainbow loading once we have a valid operating mode
  if ( rainbow_loading_effect_ )
    rainbow_loading_effect_->deactivate();
}

void AthenaStatusLedDriver::onBatteryStatus(
    const athena_firmware_interface_msgs::msg::BatteryStatus::SharedPtr msg )
{
  // Update power supply effect
  power_supply_effect_->setOnPowerSupply(
      msg->power_source == athena_firmware_interface_msgs::msg::BatteryStatus::POWER_SUPPLY );

  // Update battery pulse effect
  std::array<uint16_t, BatteryPulseEffect::CELLS_PER_BATTERY> cells1{};
  std::array<uint16_t, BatteryPulseEffect::CELLS_PER_BATTERY> cells2{};
  for ( size_t i = 0; i < BatteryPulseEffect::CELLS_PER_BATTERY; ++i ) {
    cells1[i] = msg->cell_voltages_battery1_mv[i];
    cells2[i] = msg->cell_voltages_battery2_mv[i];
  }
  battery_pulse_effect_->updateBatteryState( cells1, cells2 );
}

void AthenaStatusLedDriver::onSetSpotLight(
    const std::shared_ptr<athena_status_led_driver_msgs::srv::SetSpotLight::Request> request,
    std::shared_ptr<athena_status_led_driver_msgs::srv::SetSpotLight::Response> response )
{
  RCLCPP_INFO( get_logger(), "SetSpotLight: enable=%d, brightness=%.2f, dir=%.1f°, width=%.1f°",
               request->enable, request->brightness, request->direction_deg, request->width_deg );

  spot_light_effect_->setSpotLight( request->enable, request->brightness, request->direction_deg,
                                    request->width_deg );
}

} // namespace athena_status_led_driver

int main( int argc, char *argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin(
      std::make_shared<athena_status_led_driver::AthenaStatusLedDriver>()->get_node_base_interface() );
  rclcpp::shutdown();
  return 0;
}
