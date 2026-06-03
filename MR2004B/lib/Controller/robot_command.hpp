#pragma once

#include <stdint.h>
#include "xbox_controller.hpp"

namespace robo {

struct RobotCommand {
    bool controller_connected = false;
    bool failsafe_active = true;

    float lateral = 0.0f;
    bool precision_active = false;

    // One-shot actions.
    bool fire_solenoid = false;
    bool increase_max_speed = false;
    bool decrease_max_speed = false;
    bool jog_left = false;
    bool jog_right = false;
    bool return_center = false;
    bool rehome_request = false;
    bool arm_request = false;

    // Hold action. True while B is held.
    bool emergency_stop = false;
};

class RobotCommandMapper {
public:
    static void init();

    static RobotCommand update(const xbox::State& state, bool failsafe_active);

    static void setTriggerDeadzone(float dz);
    static float getTriggerDeadzone();

    static void setLeftStickDeadzone(float dz);
    static float getLeftStickDeadzone();

    static void setPrecisionScale(float scale);
    static float getPrecisionScale();

    static void setRehomeHoldMs(uint32_t hold_ms);
    static uint32_t getRehomeHoldMs();

private:
    static bool risingEdge(bool current, bool& previous);
    static float applyDeadzone(float value, float deadzone);

    static bool prev_lb;
    static bool prev_rb;
    static bool prev_dpad_up;
    static bool prev_dpad_down;
    static bool prev_dpad_left;
    static bool prev_dpad_right;
    static bool prev_start;
    static bool prev_a;

    static bool back_was_held;
    static bool back_hold_fired;
    static uint32_t back_hold_start_ms;

    static float trigger_deadzone;
    static float left_stick_deadzone;
    static float precision_scale;
    static uint32_t rehome_hold_ms;
};

} 
