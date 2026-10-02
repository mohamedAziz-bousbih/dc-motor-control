cmake_minimum_required(VERSION 3.16)
project(dc_motor_control
        DESCRIPTION "motorctl: portable C11 controllers for a DC servo, with golden-vector tests"
        LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(MOTORCTL_WERROR "Treat compiler warnings as errors" OFF)
option(MOTORCTL_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

if(MOTORCTL_SANITIZE)
  # Any finding aborts the test program, so CTest reports it as a failure.
  add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all
                      -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

add_library(motorctl_warnings INTERFACE)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(motorctl_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes -Wmissing-prototypes
    -Wcast-align -Wwrite-strings -Wundef)
  if(MOTORCTL_WERROR)
    target_compile_options(motorctl_warnings INTERFACE -Werror)
  endif()
elseif(MSVC)
  target_compile_options(motorctl_warnings INTERFACE /W4)
endif()

# The library proper. An OBJECT library: the test programs link the object
# files directly, so no archiver is needed (this keeps the build working with
# `zig cc` on hosts without binutils). The firmware builds the same sources
# through PlatformIO (firmware/platformio.ini, lib_extra_dirs).
add_library(motorctl OBJECT
  lib/motorctl/src/mc_cmd.c
  lib/motorctl/src/mc_encoder.c
  lib/motorctl/src/mc_filter.c
  lib/motorctl/src/mc_pid.c
  lib/motorctl/src/mc_servo.c
  lib/motorctl/src/mc_ssc.c)
target_include_directories(motorctl PUBLIC lib/motorctl/include)
target_link_libraries(motorctl PRIVATE motorctl_warnings)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  # The ESP32 FPU is single precision; an accidental double in the per-sample
  # code would be emulated in software. Make every promotion visible.
  target_compile_options(motorctl PRIVATE -Wdouble-promotion -Wfloat-conversion)
endif()
if(UNIX)
  target_link_libraries(motorctl PUBLIC m)
endif()

include(CTest)
if(BUILD_TESTING)
  # Golden-vector reader shared by the replay tests.
  add_library(golden OBJECT tests/c/golden.c)
  target_include_directories(golden PUBLIC tests/c)
  target_compile_definitions(golden PRIVATE GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/golden")
  target_link_libraries(golden PRIVATE motorctl_warnings)

  foreach(name IN ITEMS test_filter test_pid test_ssc test_servo test_encoder test_cmd)
    add_executable(${name} tests/c/${name}.c)
    # firmware/include holds the generated controller_params.h: the tests use
    # exactly the configuration the firmware is built with.
    target_include_directories(${name} PRIVATE firmware/include)
    target_link_libraries(${name} PRIVATE motorctl golden motorctl_warnings)
    add_test(NAME ${name} COMMAND ${name})
  endforeach()
endif()
