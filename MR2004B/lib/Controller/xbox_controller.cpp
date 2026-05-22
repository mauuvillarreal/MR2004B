#include "xbox_controller.hpp"

#include <math.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"

namespace xbox {

static State g_state;
static mutex_t g_state_mutex;
static float g_deadzone = 0.08f;

void XboxController::init() {
    mutex_init(&g_state_mutex);

    mutex_enter_blocking(&g_state_mutex);
    g_state = State{};
    mutex_exit(&g_state_mutex);
}

State XboxController::getState() {
    mutex_enter_blocking(&g_state_mutex);
    State copy = g_state;
    mutex_exit(&g_state_mutex);
    return copy;
}

bool XboxController::isConnected() {
    return getState().connected;
}

bool XboxController::isFailsafeActive(uint32_t timeout_ms) {
    State s = getState();

    if (!s.connected) {
        return true;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    return (now - s.last_update_ms) > timeout_ms;
}

void XboxController::setDeadzone(float dz) {
    if (dz < 0.0f) dz = 0.0f;
    if (dz > 0.5f) dz = 0.5f;
    g_deadzone = dz;
}

bool XboxController::pressed(Button b) {
    State s = getState();
    return (s.buttons & static_cast<uint32_t>(b)) != 0;
}

void XboxController::onConnected() {
    mutex_enter_blocking(&g_state_mutex);
    g_state.connected = true;
    g_state.last_update_ms = to_ms_since_boot(get_absolute_time());
    mutex_exit(&g_state_mutex);
}

void XboxController::onDisconnected() {
    mutex_enter_blocking(&g_state_mutex);
    g_state = State{};
    g_state.connected = false;
    mutex_exit(&g_state_mutex);
}

float XboxController::normalizeAxis(int value) {
    // Clamp to protect against controller-specific overflow.
    float v = static_cast<float>(value) / 512.0f;

    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;

    return applyDeadzone(v);
}

float XboxController::normalizeTrigger(int value) {
    // Bluepad32 brake/throttle (0 to 1023).
    float v = static_cast<float>(value) / 1023.0f;

    if (v > 1.0f) v = 1.0f;
    if (v < 0.0f) v = 0.0f;

    return v;
}

float XboxController::applyDeadzone(float value) {
    if (fabsf(value) < g_deadzone) {
        return 0.0f;
    }

    return value;
}

void XboxController::onGamepadData(
    int axis_x,
    int axis_y,
    int axis_rx,
    int axis_ry,
    int brake,
    int throttle,
    uint32_t buttons,
    uint32_t misc_buttons,
    uint8_t dpad
) {
    (void)misc_buttons;

    uint32_t mapped_buttons = buttons;

    // D-pad mapping used by Bluepad32.
    if (dpad & 0x01) mapped_buttons |= BTN_DPAD_UP;
    if (dpad & 0x02) mapped_buttons |= BTN_DPAD_DOWN;
    if (dpad & 0x04) mapped_buttons |= BTN_DPAD_RIGHT;
    if (dpad & 0x08) mapped_buttons |= BTN_DPAD_LEFT;

    mutex_enter_blocking(&g_state_mutex);

    g_state.connected = true;

    g_state.lx = normalizeAxis(axis_x);
    g_state.ly = -normalizeAxis(axis_y);
    g_state.rx = normalizeAxis(axis_rx);
    g_state.ry = -normalizeAxis(axis_ry);

    g_state.lt = normalizeTrigger(brake);
    g_state.rt = normalizeTrigger(throttle);

    g_state.buttons = mapped_buttons;
    g_state.last_update_ms = to_ms_since_boot(get_absolute_time());

    mutex_exit(&g_state_mutex);
}

} 

extern "C" {

void xbox_cpp_on_connected(void) {
    xbox::XboxController::onConnected();
}

void xbox_cpp_on_disconnected(void) {
    xbox::XboxController::onDisconnected();
}

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
) {
    xbox::XboxController::onGamepadData(
        axis_x,
        axis_y,
        axis_rx,
        axis_ry,
        brake,
        throttle,
        buttons,
        misc_buttons,
        dpad
    );
}

}
