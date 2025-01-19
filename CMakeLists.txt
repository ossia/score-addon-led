cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# Only Linux allows raw SPI access so far
find_path(LINUX_HEADERS_INCLUDE_DIR linux/spi/spidev.h)
if(NOT LINUX_HEADERS_INCLUDE_DIR)
  return()
endif()
if(APPLE OR EMSCRIPTEN OR WIN32 OR BSD)
  return()
endif()
if(NOT TARGET score_lib_base)
  include("${SCORE_SOURCE_DIR}/cmake/ScoreExternalAddon.cmake")
endif()

project(score_addon_led LANGUAGES CXX)

# General initialization
score_common_setup()

# Creation of the library
add_library(score_addon_led
  Led/Device.cpp
  Led/Device.hpp
  Led/ProtocolFactory.cpp
  Led/ProtocolFactory.hpp
  Led/ProtocolSettingsWidget.cpp
  Led/ProtocolSettingsWidget.hpp
  Led/SpecificSettings.hpp
  Led/SpecificSettingsSerialization.cpp

  score_addon_led.hpp
  score_addon_led.cpp
)

# Link
target_link_libraries(score_addon_led PUBLIC score_plugin_engine)

# Target-specific options
setup_score_plugin(score_addon_led)


# avnd_score_plugin_init(
#   BASE_TARGET score_addon_ledfx
# )
#
# avnd_score_plugin_add(
#   BASE_TARGET score_addon_ledfx
#   SOURCES
#     LedProcesses/LedFX.hpp
#   TARGET ledfx
#   MAIN_CLASS LedFX
#   NAMESPACE Led
# )
# avnd_score_plugin_finalize(
#   BASE_TARGET score_addon_ledfx
#   PLUGIN_VERSION 1
#   PLUGIN_UUID "706e49b7-6e22-4a4d-9f91-3c2a7563c072"
# )
