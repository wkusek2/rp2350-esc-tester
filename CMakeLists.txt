cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2 CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(esc_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(esc_test main.c dshot.c kiss.c fourway.c imu.c press.c)
pico_generate_pio_header(esc_test ${CMAKE_CURRENT_LIST_DIR}/dshot_bidir.pio)
target_link_libraries(esc_test pico_stdlib pico_multicore hardware_pio hardware_dma hardware_adc hardware_uart hardware_irq hardware_clocks hardware_i2c)
# stdio tylko przez USB - UART0 (GPIO0/1) jest zajety przez telemetrie ESC #1
pico_enable_stdio_usb(esc_test 1)
pico_enable_stdio_uart(esc_test 0)
# BLHeliSuite32 ustawia 1200 baud przy 'reboot FC' - bez tego Pico wskoczylby w BOOTSEL
target_compile_definitions(esc_test PRIVATE PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=0)
pico_add_extra_outputs(esc_test)

# -DRAM_BUILD=ON: obraz do SRAM (picotool load -x), gdy flash nie dziala
option(RAM_BUILD "Build no_flash image" OFF)
if (RAM_BUILD)
    pico_set_binary_type(esc_test no_flash)
endif()
