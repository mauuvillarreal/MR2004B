#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void xbox_cpp_on_connected(void);
void xbox_cpp_on_disconnected(void);
void xbox_cpp_on_gamepad_data(
    int axis_x,
    int axis_y,
    int axis_rx,
    int axis_ry,
    int brake,
    int throttle,
    uint32_t buttons,
    uint32_t misc_buttons,
    uint8_t dpad
);

#ifdef __cplusplus
}
#endif
