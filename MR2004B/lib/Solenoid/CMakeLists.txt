add_library(solenoid
    solenoid.cpp
)

target_include_directories(solenoid PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(solenoid PUBLIC
    pico_stdlib
    pico_time
    hardware_gpio
)
