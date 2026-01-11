# Executable
add_executable(steppertesting
    steppertesting.cpp
)
target_link_libraries(steppertesting
    pico_stdlib
    hardware_spi
    hardware_gpio
    TMC429
    hardware_pwm
    hardware_clocks
)

target_compile_features(steppertesting PUBLIC cxx_std_17)

pico_add_extra_outputs(steppertesting)