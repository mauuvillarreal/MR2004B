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
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(MR2004B C CXX ASM)

pico_sdk_init()

include_directories(${CMAKE_CURRENT_LIST_DIR}/src)

set(BLUEPAD32_ROOT ${CMAKE_CURRENT_LIST_DIR}/external/bluepad32)
set(BLUEPAD32_COMPONENT_DIR ${BLUEPAD32_ROOT}/src/components/bluepad32)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/Controller)
    if(NOT EXISTS ${BLUEPAD32_COMPONENT_DIR}/CMakeLists.txt)
        message(FATAL_ERROR
            "lib/Controller exists, but Bluepad32 was not found at:\n"
            "  ${BLUEPAD32_ROOT}\n"
            "Expected CMake file:\n"
            "  ${BLUEPAD32_COMPONENT_DIR}/CMakeLists.txt\n"
            "Install with:\n"
            "  git clone --recursive https://github.com/ricardoquesada/bluepad32.git external/bluepad32"
        )
    endif()

    add_subdirectory(${BLUEPAD32_COMPONENT_DIR} bluepad32)
endif()

add_subdirectory(lib/TMC2209)
add_subdirectory(lib/MPU)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/VL53L0X/CMakeLists.txt)
    add_subdirectory(lib/VL53L0X)
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/ExternalLED/CMakeLists.txt)
    add_subdirectory(lib/ExternalLED)
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/Solenoid/CMakeLists.txt)
    add_subdirectory(lib/Solenoid)
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/Controller/CMakeLists.txt)
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
    ${CMAKE_CURRENT_LIST_DIR}/lib/TMC2209
    ${CMAKE_CURRENT_LIST_DIR}/lib/MPU
)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/VL53L0X)
    target_include_directories(MR2004B PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib/VL53L0X)
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/ExternalLED)
    target_include_directories(MR2004B PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib/ExternalLED)
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/Solenoid)
    target_include_directories(MR2004B PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib/Solenoid)
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/Controller)
    target_include_directories(MR2004B PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/lib/Controller
        ${BLUEPAD32_COMPONENT_DIR}/include
    )
endif()

target_link_libraries(MR2004B
    pico_stdlib
    pico_multicore

    hardware_uart
    hardware_gpio
    hardware_timer
    hardware_spi
    hardware_i2c
    pico_time
    m

    TMC2209
    MPU
)

if(TARGET vl53l0x)
    target_link_libraries(MR2004B vl53l0x)
elseif(TARGET VL53L0X)
    target_link_libraries(MR2004B VL53L0X)
endif()

if(TARGET external_led)
    target_link_libraries(MR2004B external_led)
elseif(TARGET ExternalLED)
    target_link_libraries(MR2004B ExternalLED)
endif()

if(TARGET Solenoid)
    target_link_libraries(MR2004B Solenoid)
elseif(TARGET solenoid)
    target_link_libraries(MR2004B solenoid)
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/Controller)
    target_link_libraries(MR2004B
        pico_cyw43_arch_none
        pico_btstack_classic
        pico_btstack_ble
        pico_btstack_cyw43
        bluepad32
    )

    if(TARGET Controller)
        target_link_libraries(MR2004B Controller)
    elseif(TARGET controller)
        target_link_libraries(MR2004B controller)
    endif()
endif()

pico_add_extra_outputs(MR2004B)
