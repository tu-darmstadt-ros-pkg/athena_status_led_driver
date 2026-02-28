#ifndef ATHENA_STATUS_LED_DRIVER_MOCK_TRANSPORT_HPP
#define ATHENA_STATUS_LED_DRIVER_MOCK_TRANSPORT_HPP

#include "athena_status_led_driver/led_transport.hpp"
#include <vector>

namespace athena_status_led_driver
{

/**
 * @brief Mock transport for unit testing.
 *
 * Records every frame sent so tests can assert on pixel colors.
 */
class MockTransport : public LedTransport
{
public:
  bool open() override
  {
    open_ = true;
    return true;
  }

  void close() override { open_ = false; }

  bool isOpen() const override { return open_; }

  bool send( const std::vector<Color> &leds ) override
  {
    last_frame_ = leds;
    frame_count_++;
    return true;
  }

  /// The most recently sent frame
  const std::vector<Color> &lastFrame() const { return last_frame_; }

  /// Total number of frames sent
  size_t frameCount() const { return frame_count_; }

  /// Reset recorded state
  void reset()
  {
    last_frame_.clear();
    frame_count_ = 0;
  }

private:
  bool open_ = false;
  std::vector<Color> last_frame_;
  size_t frame_count_ = 0;
};

} // namespace athena_status_led_driver

#endif // ATHENA_STATUS_LED_DRIVER_MOCK_TRANSPORT_HPP
