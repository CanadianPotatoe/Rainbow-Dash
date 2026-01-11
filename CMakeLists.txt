cmake_minimum_required(VERSION 3.13)

# 1. Pull in SDK (Make sure pico_sdk_import.cmake is in this folder!)
include(pico_sdk_import.cmake)

project(Rainbow-Dash C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 2. Initialize the SDK
pico_sdk_init()

# 3. Standard Pico compiler flags
add_compile_options(-Wall
        -Wno-format
        -Wno-unused-function
)

if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# 4. Add your project subdirectories
add_subdirectory(BNO08x_Pico_Library)
add_subdirectory(TMC429)
add_subdirectory(blink)
add_subdirectory(imutesting)
add_subdirectory(steppertesting)