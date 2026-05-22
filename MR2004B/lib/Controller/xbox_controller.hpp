#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace xbox {

enum Button : uint32_t {
    BTN_A          = 1u << 0,
    BTN_B          = 1u << 1,
    BTN_X          = 1u << 2,
    BTN_Y          = 1u << 3,
    BTN_LB         = 1u << 4,
    BTN_RB         = 1u << 5,
    BTN_BACK       = 1u << 6,
    BTN_START      = 1u << 7,
    BTN_THUMB_L    = 1u << 8,
    BTN_THUMB_R    = 1u << 9,
    BTN_DPAD_UP    = 1u << 10,
    BTN_DPAD_DOWN  = 1u << 11,
    BTN_DPAD_LEFT  = 1u << 12,
    BTN_DPAD_RIGHT = 1u << 13,
};

struct State {
    bool connected = false;

    // Normalized axes: -1.0f to +1.0f
    float lx = 0.0f;
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;

    // Triggers: 0.0f to 1.0f
    float lt = 0.0f;
    float rt = 0.0f;

    uint32_t buttons = 0;

    // Updated whenever fresh controller data arrives.
    uint32_t last_update_ms = 0;
};

class XboxController {
public:
    static void init();

    static State getState();

    static bool isConnected();

    // True when controller is disconnected or packets stopped arriving.
    static bool isFailsafeActive(uint32_t timeout_ms = 250);

    static void setDeadzone(float dz);

    static bool pressed(Button b);

    static void onConnected();
    static void onDisconnected();
    static void onGamepadData(
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

private:
    static float normalizeAxis(int value);
    static float normalizeTrigger(int value);
    static float applyDeadzone(float value);
};

}
