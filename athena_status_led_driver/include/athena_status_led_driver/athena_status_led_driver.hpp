#ifndef ATHENA_STATUS_LED_DRIVER_ATHENA_STATUS_LED_DRIVER_HPP
#define ATHENA_STATUS_LED_DRIVER_ATHENA_STATUS_LED_DRIVER_HPP

#include <memory>
#include <string>

#include <hector_ros2_utils/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <athena_firmware_interface_msgs/msg/battery_status.hpp>

#include <athena_status_led_driver_msgs/srv/set_spot_light.hpp>

#include "athena_status_led_driver/led_ring_controller.hpp"
#include "athena_status_led_driver/led_transport.hpp"

namespace athena_status_led_driver
{

class OperatingModeEffect;
class BatteryPulseEffect;
class PowerSupplyEffect;
class SpotLightEffect;
class RainbowLoadingEffect;

class AthenaStatusLedDriver : public hector::Node
{
public:
  AthenaStatusLedDriver();
  ~AthenaStatusLedDriver() override;

private:
  void setup();
  void cleanUp();

  /// Timer callback for animation updates
  void onTimer();

  /// Subscriber callbacks
  void onOperatingMode(const std_msgs::msg::String::SharedPtr msg);
  void onBatteryStatus(const athena_firmware_interface_msgs::msg::BatteryStatus::SharedPtr msg);

  /// Service callback
  void onSetSpotLight(
    const std::shared_ptr<athena_status_led_driver_msgs::srv::SetSpotLight::Request> request,
    std::shared_ptr<athena_status_led_driver_msgs::srv::SetSpotLight::Response> response);

  // Parameters
  int led_count_ = 110;
  std::string spi_device_ = "/dev/spidev3.0";
  int spi_speed_hz_ = 6500000;
  double update_rate_hz_ = 30.0;
  double global_brightness_ = 1.0;
  bool simulate_ = false;

  // Controller and transport
  std::shared_ptr<LedTransport> transport_;
  std::unique_ptr<LedRingController> controller_;

  // Effects
  std::shared_ptr<OperatingModeEffect> operating_mode_effect_;
  std::shared_ptr<BatteryPulseEffect> battery_pulse_effect_;
  std::shared_ptr<PowerSupplyEffect> power_supply_effect_;
  std::shared_ptr<SpotLightEffect> spot_light_effect_;
  std::shared_ptr<RainbowLoadingEffect> rainbow_loading_effect_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operating_mode_sub_;
  rclcpp::Subscription<athena_firmware_interface_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Service<athena_status_led_driver_msgs::srv::SetSpotLight>::SharedPtr set_spot_light_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Timing
  rclcpp::Time last_tick_time_;
};

}  // namespace athena_status_led_driver

#endif  // ATHENA_STATUS_LED_DRIVER_ATHENA_STATUS_LED_DRIVER_HPP
