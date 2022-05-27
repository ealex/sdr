# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(PICO_DEFAULT_BINARY_TYPE copy_to_ram)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/alexandru/Desktop/work/rp2040/pico-sdk/")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(uSDR C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# add external libraries
add_subdirectory(pico-ss-oled)

# Add executable. Default name is the project name, version 0.1
add_executable(uSDR uSDR.c lcd.c dsp.c hmi.c si5351.c)

pico_set_program_name(uSDR "uSDR")
pico_set_program_version(uSDR "0.1")

# Pull in our pico_stdlib which aggregates commonly used features
target_link_libraries(uSDR pico_stdlib)
target_compile_options(uSDR PRIVATE -Wall -Wextra)

# Disable uart output, enable usb output
pico_enable_stdio_uart(uSDR 1)
pico_enable_stdio_usb(uSDR 0)


# Add any user requested libraries
target_link_libraries(
		uSDR
	    pico_stdlib
		pico_multicore
		hardware_i2c
		hardware_pwm
		hardware_gpio
		hardware_timer
		hardware_clocks
		hardware_pll
		hardware_adc
		pico_ss_oled
	)

pico_add_extra_outputs(uSDR)

