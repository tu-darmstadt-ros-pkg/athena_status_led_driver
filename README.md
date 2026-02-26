# Athena Status LED Driver

ROS 2 driver for the Athena 110-LED WS2812B status light ring.
It uses SPI hardware (via `/dev/spidev3.0`) on the Radxa Zero 3E, rendering a priority-composited pipeline of visual effects.

## Setup & Hardware Prerequisites

1. **Hardware Connection:**
   The WS2812B data line must be connected to the Radxa's SPI MOSI pin (typically SPI3 MOSI on pin 19).

2. **Enable SPI Overlay:**
   Ensure the `spidev3.0` device is enabled in the Radxa device tree overlays so it appears as `/dev/spidev3.0`.

3. **Permissions (udev rule):**
   The ROS node needs permission to write to the SPI device.
   Create a new file `/etc/udev/rules.d/99-spidev.rules` with the following content:
   ```text
   SUBSYSTEM=="spidev", KERNEL=="spidev3.0", GROUP="dialout", MODE="0660"
   ```
   *Note: Ensure the user running the ROS node is part of the `dialout` group (`sudo usermod -aG dialout $USER`).*
   Reload rules or reboot for it to take effect: `sudo udevadm control --reload-rules && sudo udevadm trigger`

## Usage

### Run the node
```bash
ros2 run athena_status_led_driver athena_status_led_driver_node
```

### ROS Interface

*   **Subscribers:**
    *   `/operating_mode` (`std_msgs/msg/String`): Expected values: `autonomous` (Blue), `driving` (Yellow-Orange), `manipulation` (Red-Orange), `safe` (Green).
    *   `/battery` (`athena_firmware_interface_msgs/msg/BatteryStatus`): Reads `power_source` and cell voltages.

*   **Services:**
    *   `~/set_spot_light` (`athena_status_led_driver_msgs/srv/SetSpotLight`): Controls a directional white arc.
        ```bash
        # Example: turn on an 80-degree wide arc aiming at 90 degrees at 50% brightness
        ros2 service call /athena_status_led_driver/set_spot_light athena_status_led_driver_msgs/srv/SetSpotLight "{enable: true, brightness: 0.5, direction_deg: 90.0, width_deg: 80.0}"
        ```

### Effect Pipeline (Priority Order)

Effects are rendered in the order they are added in the controller (lowest to highest priority visually). "Fill" effects write all pixels. "Blend" effects selectively modify pixels.

1.  **Rainbow Loading (Fill):** Spinning HSV rainbow active on startup. Deactivated permanently when the first `operating_mode` message arrives.
2.  **Operating Mode (Fill):** Static color based on current mode. Overwrites the rainbow.
3.  **Battery Pulse (Blend):** Red sine wave pulsing when *both* batteries have a cell under 3.8V.
4.  **Power Supply Chase (Blend):** A few green LEDs circling when connected to external power.
5.  **Spot Light Mode (Blend):** Directional white illumination arc controlled via service.

## How to Extend (Adding a Custom Effect)

The effect system is designed for easy extension without modifying the core controller logic.

1.  **Create the effect class:**
    Inherit from `athena_status_led_driver::LedEffect` and implement the 3 core methods:
    *   `isActive()`: Should the effect render on this frame?
    *   `update(double dt)`: Advance your animation state (e.g. phase, offset) by `dt` seconds.
    *   `render(std::vector<Color>& pixels)`: Modify the `pixels` array. Either overwrite all pixels (fill) or selectively change/blend specific pixels based on your logic.

    *Tip: Inspect `include/athena_status_led_driver/battery_pulse_effect.hpp` for a blending example, or `rainbow_loading_effect.hpp` for a fill example.*

2.  **Add it to the Node:**
    *   Include your header in `src/athena_status_led_driver.cpp`.
    *   Instantiate your effect in `AthenaStatusLedDriver::setup()`.
    *   Add it to the controller list using `controller_->addEffect(my_new_effect_);`.
    *   *Note on ordering:* Add it *before* effects that should visually draw on top of it, and *after* effects it should draw over or overwrite.
