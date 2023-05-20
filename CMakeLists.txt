cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)


project(hid_controller_to_SBUS C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()



add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

set(FAMILY rp2040)
set(BOARD pico_sdk)

add_executable(hid_controller_to_SBUS)

# Source Code
target_sources(hid_controller_to_SBUS PUBLIC main.c)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(hid_controller_to_SBUS PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(hid_controller_to_SBUS PUBLIC pico_stdlib tinyusb_host tinyusb_board pico_stdlib pico_multicore hardware_pwm)

pico_add_extra_outputs(hid_controller_to_SBUS)

pico_enable_stdio_usb(hid_controller_to_SBUS 0)
pico_enable_stdio_uart(hid_controller_to_SBUS 1)
