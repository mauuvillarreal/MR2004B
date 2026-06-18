add_library(external_led STATIC
    external_led.cpp
)

target_include_directories(external_led PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(external_led PUBLIC
    pico_stdlib
    hardware_gpio
    hardware_pwm
)
