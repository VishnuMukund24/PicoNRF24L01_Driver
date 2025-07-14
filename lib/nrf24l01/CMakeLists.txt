add_library(nrf24l01_lib 
    nrf24l01.cpp
    nrf24l01.h
    )

target_include_directories(nrf24l01_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

target_link_libraries(nrf24l01_lib
    pico_stdlib
    hardware_spi
    hardware_gpio
    )