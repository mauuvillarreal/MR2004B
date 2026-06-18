#include "robot_command.hpp"

#include <math.h>
#include "pico/stdlib.h"

namespace robo {

bool RobotCommandMapper::prev_lb = false;
bool RobotCommandMapper::prev_rb = false;
bool RobotCommandMapper::prev_dpad_up = false;
bool RobotCommandMapper::prev_dpad_down = false;
bool RobotCommandMapper::prev_dpad_left = false;
bool RobotCommandMapper::prev_dpad_right = false;
bool RobotCommandMapper::prev_start = false;
bool RobotCommandMapper::prev_a = false;
bool RobotCommandMapper::prev_y = false;

bool RobotCommandMapper::back_was_held = false;
bool RobotCommandMapper::back_hold_fired = false;
uint32_t RobotCommandMapper::back_hold_start_ms = 0;

float RobotCommandMapper::trigger_deadzone = 0.03f;
float RobotCommandMapper::left_stick_deadzone = 0.08f;
float RobotCommandMapper::precision_scale = 0.25f;
uint32_t RobotCommandMapper::rehome_hold_ms = 2000;

void RobotCommandMapper::init() {
    prev_lb = false;
    prev_rb = false;
    prev_dpad_up = false;
    prev_dpad_down = false;
    prev_dpad_left = false;
    prev_dpad_right = false;
    prev_start = false;
    prev_a = false;
    prev_y = false;

    back_was_held = false;
    back_hold_fired = false;
    back_hold_start_ms = 0;
}

bool RobotCommandMapper::risingEdge(bool current, bool& previous) {
    bool result = current && !previous;
    previous = current;
    return result;
}

float RobotCommandMapper::applyDeadzone(float value, float deadzone) {
    if (fabsf(value) < deadzone) {
        return 0.0f;
    }

    return value;
}

void RobotCommandMapper::setTriggerDeadzone(float dz) {
    if (dz < 0.0f) dz = 0.0f;
    if (dz > 0.5f) dz = 0.5f;
    trigger_deadzone = dz;
}

float RobotCommandMapper::getTriggerDeadzone() {
    return trigger_deadzone;
}

void RobotCommandMapper::setLeftStickDeadzone(float dz) {
    if (dz < 0.0f) dz = 0.0f;
    if (dz > 0.5f) dz = 0.5f;
    left_stick_deadzone = dz;
}

float RobotCommandMapper::getLeftStickDeadzone() {
    return left_stick_deadzone;
}

void RobotCommandMapper::setPrecisionScale(float scale) {
    if (scale < 0.05f) scale = 0.05f;
    if (scale > 1.0f) scale = 1.0f;
    precision_scale = scale;
}

float RobotCommandMapper::getPrecisionScale() {
    return precision_scale;
}

void RobotCommandMapper::setRehomeHoldMs(uint32_t hold_ms) {
    if (hold_ms < 500u) hold_ms = 500u;
    if (hold_ms > 10000u) hold_ms = 10000u;
    rehome_hold_ms = hold_ms;
}

uint32_t RobotCommandMapper::getRehomeHoldMs() {
    return rehome_hold_ms;
}

RobotCommand RobotCommandMapper::update(const xbox::State& state, bool failsafe_active) {
    RobotCommand cmd;

    cmd.controller_connected = state.connected;
    cmd.failsafe_active = failsafe_active;

    if (failsafe_active || !state.connected) {
        cmd.lateral = 0.0f;
        cmd.emergency_stop = true;

        // Reset edge/hold state while disconnected so reconnecting starts.
        init();
        return cmd;
    }

    // Left joystick X is the primary fast movement control.
    float stick_lateral = applyDeadzone(state.lx, left_stick_deadzone);

    // RT/LT are slow precision movement: RT - LT.
    float rt = state.rt;
    float lt = state.lt;

    if (rt < trigger_deadzone) rt = 0.0f;
    if (lt < trigger_deadzone) lt = 0.0f;

    float trigger_lateral = rt - lt;

    if (stick_lateral != 0.0f) {
        cmd.lateral = stick_lateral;
        cmd.precision_active = false;
    } else if (fabsf(trigger_lateral) > trigger_deadzone) {
        cmd.lateral = trigger_lateral;
        cmd.precision_active = true;
    } else {
        cmd.lateral = 0.0f;
        cmd.precision_active = false;
    }

    if (cmd.lateral > 1.0f) cmd.lateral = 1.0f;
    if (cmd.lateral < -1.0f) cmd.lateral = -1.0f;

    bool lb_now = (state.buttons & xbox::BTN_LB) != 0;
    bool rb_now = (state.buttons & xbox::BTN_RB) != 0;
    bool dpad_up_now = (state.buttons & xbox::BTN_DPAD_UP) != 0;
    bool dpad_down_now = (state.buttons & xbox::BTN_DPAD_DOWN) != 0;
    bool dpad_left_now = (state.buttons & xbox::BTN_DPAD_LEFT) != 0;
    bool dpad_right_now = (state.buttons & xbox::BTN_DPAD_RIGHT) != 0;
    bool start_now = (state.buttons & xbox::BTN_START) != 0;
    bool back_now = (state.buttons & xbox::BTN_BACK) != 0;
    bool a_now = (state.buttons & xbox::BTN_A) != 0;
    bool b_now = (state.buttons & xbox::BTN_B) != 0;
    bool y_now = (state.buttons & xbox::BTN_Y) != 0;

    // LB is kick button. 
    bool lb_fire = risingEdge(lb_now, prev_lb);
    bool rb_fire = risingEdge(rb_now, prev_rb);
    cmd.fire_solenoid = lb_fire || rb_fire;

    cmd.increase_max_speed = risingEdge(dpad_up_now, prev_dpad_up);
    cmd.decrease_max_speed = risingEdge(dpad_down_now, prev_dpad_down);
    cmd.jog_left = risingEdge(dpad_left_now, prev_dpad_left);
    cmd.jog_right = risingEdge(dpad_right_now, prev_dpad_right);
    cmd.return_center = risingEdge(start_now, prev_start);
    cmd.arm_request = risingEdge(a_now, prev_a);
    cmd.toggle_auto_kick = risingEdge(y_now, prev_y);

    // B is immediate stop/disarm.
    cmd.emergency_stop = b_now;

    // View / Back requires a hold to request re-home.
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (back_now) {
        if (!back_was_held) {
            back_was_held = true;
            back_hold_fired = false;
            back_hold_start_ms = now;
        }

        if (!back_hold_fired && (uint32_t)(now - back_hold_start_ms) >= rehome_hold_ms) {
            cmd.rehome_request = true;
            back_hold_fired = true;
        }
    } else {
        back_was_held = false;
        back_hold_fired = false;
        back_hold_start_ms = 0;
    }

    return cmd;
}

}  