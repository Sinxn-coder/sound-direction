cmake_minimum_required(VERSION 3.16)
project(SoundDirection VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8)
  add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# sdl_core: the portable direction engine. No file I/O, no OS dependencies.
# This is the part that will be reused unchanged on the ESP32-S3.
# -----------------------------------------------------------------------------
add_library(sdl_core STATIC
  src/utils/DspUtils.cpp
  src/direction/CrossCorrelation.cpp
  src/direction/DirectionEngine.cpp
  src/direction/TemporalSmoother.cpp
)
target_include_directories(sdl_core PUBLIC src)

# Build the core with the ESP32 numeric settings (float accumulators) so the
# desktop tests can validate the embedded code path.
option(SDL_CORE_FLOAT_ACCUM "Use float accumulators in the core (as on ESP32)" OFF)
if(SDL_CORE_FLOAT_ACCUM)
  target_compile_definitions(sdl_core PUBLIC SDL_CORE_FLOAT_ACCUM)
endif()

# -----------------------------------------------------------------------------
# sdl_desktop: laptop-only pieces (WAV files, simulation, console output).
# -----------------------------------------------------------------------------
add_library(sdl_desktop STATIC
  src/audio/WavFile.cpp
  src/audio/BufferStereoSource.cpp
  src/simulation/BinauralSimulator.cpp
  src/simulation/SignalGenerator.cpp
  src/utils/ConsoleViz.cpp
)
target_link_libraries(sdl_desktop PUBLIC sdl_core)

add_executable(sound_direction src/main.cpp)
target_link_libraries(sound_direction PRIVATE sdl_desktop)

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
enable_testing()
add_executable(direction_tests tests/test_main.cpp)
target_link_libraries(direction_tests PRIVATE sdl_desktop)
add_test(NAME direction_tests COMMAND direction_tests)
