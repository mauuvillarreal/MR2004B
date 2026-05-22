add_library(controller
    xbox_controller.cpp
    xbox_bluepad32_platform.c
    robot_command.cpp
)

add_library(Controller ALIAS controller)

target_include_directories(controller PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# Bluepad32 is required because xbox_bluepad32_platform.c includes <uni.h>.
# The top-level CMakeLists.txt must set BLUEPAD32_ROOT and add Bluepad32 first.
if(NOT DEFINED BLUEPAD32_ROOT)
    message(FATAL_ERROR "BLUEPAD32_ROOT is not defined. Set it in the top-level CMakeLists.txt.")
endif()

if(NOT EXISTS "${BLUEPAD32_ROOT}/src/components/bluepad32/include/uni.h")
    message(FATAL_ERROR "Bluepad32 uni.h was not found. Expected: ${BLUEPAD32_ROOT}/src/components/bluepad32/include/uni.h")
endif()

target_include_directories(controller PUBLIC
    ${BLUEPAD32_ROOT}/src/components/bluepad32/include
)

target_link_libraries(controller PUBLIC
    pico_stdlib
    pico_time
    pico_sync
    pico_cyw43_arch_none
    pico_btstack_classic
    pico_btstack_cyw43
)

if(TARGET bluepad32)
    target_link_libraries(controller PUBLIC bluepad32)
endif()
