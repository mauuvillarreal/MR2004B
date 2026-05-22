#pragma once

#include <stdint.h>
#include "xbox_controller.hpp"

namespace robo {

struct RobotCommand {
    bool controller_connected = false;
    bool failsafe_active = true;

    // -1.0 = full left, +1.0 = full right
    float lateral = 0.0f;

    // One-shot trigger, not continuous hold
    bool fire_solenoid = false;

    // One-shot speed adjustment
    bool increase_max_speed = false;
    bool decrease_max_speed = false;

    bool emergency_stop = false;
};

class RobotCommandMapper {
public:
    static void init();

    static RobotCommand update(const xbox::State& state, bool failsafe_active);

private:
    static bool risingEdge(bool current, bool& previous);

    static bool prev_rb;
    static bool prev_dpad_up;
    static bool prev_dpad_down;
};

}  // namespace robo
