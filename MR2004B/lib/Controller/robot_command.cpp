#include "robot_command.hpp"

namespace robo {

bool RobotCommandMapper::prev_rb = false;
bool RobotCommandMapper::prev_dpad_up = false;
bool RobotCommandMapper::prev_dpad_down = false;

void RobotCommandMapper::init() {
    prev_rb = false;
    prev_dpad_up = false;
    prev_dpad_down = false;
}

bool RobotCommandMapper::risingEdge(bool current, bool& previous) {
    bool result = current && !previous;
    previous = current;
    return result;
}

RobotCommand RobotCommandMapper::update(const xbox::State& state, bool failsafe_active) {
    RobotCommand cmd;

    cmd.controller_connected = state.connected;
    cmd.failsafe_active = failsafe_active;

    if (failsafe_active) {
        cmd.lateral = 0.0f;
        cmd.fire_solenoid = false;
        cmd.increase_max_speed = false;
        cmd.decrease_max_speed = false;
        cmd.emergency_stop = true;
        return cmd;
    }

    float right = state.rt;
    float left = state.lt;

    // Trigger deadzone to avoid tiny accidental movement.
    if (right < 0.03f) right = 0.0f;
    if (left < 0.03f) left = 0.0f;

    // Differential trigger movement:
    // RT moves right, LT moves left.
    // Example: RT=0.50 and LT=0.25 gives +0.25 right movement.
    cmd.lateral = right - left;

    if (cmd.lateral > 1.0f) cmd.lateral = 1.0f;
    if (cmd.lateral < -1.0f) cmd.lateral = -1.0f;

    bool rb_now = (state.buttons & xbox::BTN_RB) != 0;
    bool dpad_up_now = (state.buttons & xbox::BTN_DPAD_UP) != 0;
    bool dpad_down_now = (state.buttons & xbox::BTN_DPAD_DOWN) != 0;
    bool b_now = (state.buttons & xbox::BTN_B) != 0;

    cmd.fire_solenoid = risingEdge(rb_now, prev_rb);
    cmd.increase_max_speed = risingEdge(dpad_up_now, prev_dpad_up);
    cmd.decrease_max_speed = risingEdge(dpad_down_now, prev_dpad_down);
    cmd.emergency_stop = b_now;

    if (cmd.emergency_stop) {
        cmd.lateral = 0.0f;
    }

    return cmd;
}

}  // namespace robo
