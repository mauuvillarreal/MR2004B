add_library(MPU
    mpu6050.cpp
)

target_include_directories(MPU PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(MPU
    pico_stdlib
    hardware_i2c
)
