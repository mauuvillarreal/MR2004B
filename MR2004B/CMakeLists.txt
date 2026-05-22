# Generated CMake Pico project file
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK. Must be before project().
include(pico_sdk_import.cmake)

project(MR2004B C CXX ASM)

pico_sdk_init()

# Bluepad32 location.
# Recommended layout:
#   external/bluepad32
# Or configure manually:
#   cmake -DBLUEPAD32_ROOT=/path/to/bluepad32 ..
set(BLUEPAD32_ROOT "${CMAKE_CURRENT_LIST_DIR}/external/bluepad32" CACHE PATH "Path to Bluepad32 repository")

# Local libraries. Each folder should contain its own CMakeLists.txt.
add_subdirectory(lib/TMC2209)
add_subdirectory(lib/MPU)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/lib/Solenoid/CMakeLists.txt")
    add_subdirectory(lib/Solenoid)
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/lib/Controller/CMakeLists.txt")
    if(EXISTS "${BLUEPAD32_ROOT}/src/components/bluepad32/CMakeLists.txt")
        add_subdirectory("${BLUEPAD32_ROOT}/src/components/bluepad32" "${CMAKE_BINARY_DIR}/bluepad32")
    else()
        message(WARNING "lib/Controller exists, but Bluepad32 was not found at: ${BLUEPAD32_ROOT}")
    endif()

    add_subdirectory(lib/Controller)
endif()

add_executable(MR2004B
    src/main.cpp
)

pico_set_program_name(MR2004B "MR2004B")
pico_set_program_version(MR2004B "0.1")

pico_enable_stdio_uart(MR2004B 0)
pico_enable_stdio_usb(MR2004B 1)

target_include_directories(MR2004B PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(MR2004B
    pico_stdlib
    pico_time
    pico_multicore

    hardware_uart
    hardware_gpio
    hardware_timer
    hardware_spi
    hardware_i2c

    m
)

# Link local libraries by whichever target name their module CMakeLists defines.
if(TARGET TMC2209)
    target_link_libraries(MR2004B TMC2209)
elseif(TARGET tmc2209)
    target_link_libraries(MR2004B tmc2209)
else()
    message(FATAL_ERROR "TMC2209 library target was not found. Check lib/TMC2209/CMakeLists.txt")
endif()

if(TARGET MPU)
    target_link_libraries(MR2004B MPU)
elseif(TARGET mpu)
    target_link_libraries(MR2004B mpu)
elseif(TARGET MPU6050)
    target_link_libraries(MR2004B MPU6050)
elseif(TARGET mpu6050)
    target_link_libraries(MR2004B mpu6050)
else()
    message(FATAL_ERROR "MPU library target was not found. Check lib/MPU/CMakeLists.txt")
endif()

if(TARGET Solenoid)
    target_link_libraries(MR2004B Solenoid)
elseif(TARGET solenoid)
    target_link_libraries(MR2004B solenoid)
endif()

if(TARGET Controller OR TARGET controller)
    target_link_libraries(MR2004B
        pico_cyw43_arch_none
        pico_btstack_classic
        pico_btstack_ble
        pico_btstack_cyw43
    )

    if(TARGET Controller)
        target_link_libraries(MR2004B Controller)
    else()
        target_link_libraries(MR2004B controller)
    endif()

    if(TARGET bluepad32)
        target_link_libraries(MR2004B bluepad32)
    endif()
endif()

pico_add_extra_outputs(MR2004B)
