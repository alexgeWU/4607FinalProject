# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(4607FinalProject C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(4607FinalProject 4607FinalProject.c bno08x.c imu.c )

# CEVA SH2 driver (plain C) used by bno08x.c
add_subdirectory(sh2)

pico_set_program_name(4607FinalProject "4607FinalProject")
pico_set_program_version(4607FinalProject "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(4607FinalProject 0)
pico_enable_stdio_usb(4607FinalProject 1)

# Add the standard library to the build
target_link_libraries(4607FinalProject
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(4607FinalProject PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)


# Add any user requested libraries
target_link_libraries(4607FinalProject 
        hardware_i2c
        pico_cyw43_arch_none
        sh2
        )

pico_add_extra_outputs(4607FinalProject)

