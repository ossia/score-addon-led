cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

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
