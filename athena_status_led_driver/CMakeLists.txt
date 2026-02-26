cmake_minimum_required(VERSION 3.14)
project(athena_status_led_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(hector_ros2_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(athena_firmware_interface_msgs REQUIRED)
find_package(athena_status_led_driver_msgs REQUIRED)

# --- Fetch ws2812b_spi ---
include(FetchContent)
FetchContent_Declare(
  ws2812b_spi
  GIT_REPOSITORY https://github.com/schilkp/ws2812b_spi.git
  GIT_TAG af9d9681fdc071d2763a506b1942aebbaccda670
)
FetchContent_MakeAvailable(ws2812b_spi)

# Build ws2812b as a static library
add_library(ws2812b_lib STATIC
  ${ws2812b_spi_SOURCE_DIR}/src/ws2812b.c
)
target_include_directories(ws2812b_lib PUBLIC
  ${ws2812b_spi_SOURCE_DIR}/src
)

# --- Effect sources (shared between node and tests) ---
set(EFFECT_SOURCES
  src/effects/rainbow_loading_effect.cpp
  src/effects/operating_mode_effect.cpp
  src/effects/battery_pulse_effect.cpp
  src/effects/power_supply_effect.cpp
  src/effects/spot_light_effect.cpp
)

# --- Main executable ---
add_executable(athena_status_led_driver_node
  src/athena_status_led_driver.cpp
  src/spi_transport.cpp
  src/led_ring_controller.cpp
  ${EFFECT_SOURCES}
)

target_include_directories(athena_status_led_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(athena_status_led_driver_node
  ws2812b_lib
)

ament_target_dependencies(athena_status_led_driver_node
  hector_ros2_utils
  rclcpp
  std_msgs
  athena_firmware_interface_msgs
  athena_status_led_driver_msgs
)

install(TARGETS athena_status_led_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_effects
    test/test_effects.cpp
    src/led_ring_controller.cpp
    ${EFFECT_SOURCES}
  )
  target_include_directories(test_effects PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_effects
    rclcpp
  )
endif()

ament_package()
