add_library(vl53l0x STATIC
    vl53l0x.cpp
)

target_include_directories(vl53l0x PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(vl53l0x PUBLIC
    pico_stdlib
    hardware_i2c
)
