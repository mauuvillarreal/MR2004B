add_library(TMC2209
    tmc2209.c
)

target_include_directories(TMC2209 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(TMC2209 PUBLIC
    pico_stdlib
    pico_time
    hardware_uart
    hardware_gpio
    hardware_timer
    hardware_i2c
    m
)