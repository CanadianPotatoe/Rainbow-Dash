# Executable
add_executable(imutesting
    imu.cpp
)

target_compile_features(imutesting PUBLIC cxx_std_17)

target_link_libraries(imutesting
    pico_stdlib
    hardware_i2c
    BNO08x_Pico_Library
    TMC429
)

pico_add_extra_outputs(imutesting)

