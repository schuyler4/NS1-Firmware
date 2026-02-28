cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(Newt_Scope C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME} main.c mcp41010.c calibration_memory.c ring_buffer.c)

pico_add_extra_outputs(${PROJECT_NAME})
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_generate_pio_header(${PROJECT_NAME} ../sampler.pio)

target_link_libraries(${PROJECT_NAME} pico_stdlib)
target_link_libraries(${PROJECT_NAME} pico_time)
target_link_libraries(${PROJECT_NAME} hardware_pio)
target_link_libraries(${PROJECT_NAME} hardware_clocks)
target_link_libraries(${PROJECT_NAME} hardware_dma)
target_link_libraries(${PROJECT_NAME} hardware_spi)
target_link_libraries(${PROJECT_NAME} hardware_pwm)
target_link_libraries(${PROJECT_NAME} hardware_gpio)
target_link_libraries(${PROJECT_NAME} hardware_irq)
target_link_libraries(${PROJECT_NAME} hardware_flash)
