# This file goes inside the TMC429 folder
add_library(TMC429 STATIC
    src/TMC429.cpp
)

target_include_directories(TMC429 PUBLIC
    include
)

target_link_libraries(TMC429
    pico_stdlib
    hardware_spi
    hardware_gpio
)
