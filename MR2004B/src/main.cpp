#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/time.h"

#include <btstack_run_loop.h>
#include <uni.h>

#include "tmc2209.h"
#include "solenoid.hpp"
#include "xbox_controller.hpp"
#include "xbox_bluepad32_platform.h"
#include "robot_command.hpp"

#define KICK_PULSE_MS   80

// Controller-mode speed tuning.
// g_controller_max_speed is the value changed with D-pad up/down.
static constexpr float CONTROLLER_START_MAX_SPEED = 4000.0f;
static constexpr float CONTROLLER_MIN_MAX_SPEED   = 1000.0f;
static constexpr float CONTROLLER_MAX_MAX_SPEED   = SAVE_SPEED;
static constexpr float CONTROLLER_SPEED_STEP      = 500.0f;
static constexpr float CONTROLLER_LATERAL_DEADZONE = 0.04f;
static constexpr uint32_t CONTROLLER_FAILSAFE_MS  = 300;

// Stall / step-loss recovery.
// StallGuard is still disabled in main(), exactly like your current baseline.
static volatile bool stall_flag = false;

static float g_controller_max_speed = CONTROLLER_START_MAX_SPEED;
static int   g_manual_direction = 0;   // -1 = left, +1 = right, 0 = stopped
static bool  g_manual_motion_active = false;
static bool  g_fault_latched = false;

static void on_stall(void) {
    // Called from motion_update() when StallGuard detects a stall.
    // Motor is already stopped by the time this fires.
    stall_flag = true;
    rgb_set(true, false, false); // red = fault
    buzzer_beep(80);
    printf(" !! STALL DETECTED — will require re-home\n");
}

static bool do_home(void) {
    printf("Homing...\n");
    bool ok = motion_home(false, HOMING_SPEED, 0);
    if (!ok) {
        rgb_set(true, false, false); // red = homing error
        buzzer_beep(200);
        printf("Homing FAILED — check endstop GP%d\n", ENDSTOP_PIN);
        return false;
    }

    rgb_set(false, false, true); // blue = homed
    stall_flag = false;
    g_fault_latched = false;

    printf("Homed. Usable rail: %ld – %ld steps (%.1f mm)\n",
           (long)mot_get_rail_min(),
           (long)mot_get_rail_max(),
           (mot_get_rail_max() - mot_get_rail_min()) / (float)STEPS_PER_MM);
    return true;
}

static bool wait_move(void) {
    while (motion_is_moving()) {
        motion_update();

        // If a stall fires mid-move, bail out of the wait loop.
        // The caller must not kick after an aborted move.
        if (stall_flag) return false;
    }
    return !stall_flag;
}

static bool wait_move_checked(const char *label) {
    int32_t start = motion_get_position();

    printf("\n[%s START]\n", label);
    printf("  start pos: %ld | rail_min: %ld | rail_max: %ld\n",
           (long)start,
           (long)mot_get_rail_min(),
           (long)mot_get_rail_max());

    while (motion_is_moving()) {
        motion_update();

        if (stall_flag) {
            printf("[%s ABORTED: STALL]\n", label);
            return false;
        }
    }

    int32_t end = motion_get_position();

    printf("[%s END]\n", label);
    printf("  end pos:   %ld | moved: %ld steps | %.1f mm\n",
           (long)end,
           (long)(end - start),
           (end - start) / (float)STEPS_PER_MM);

    return true;
}

static void print_pos(void) {
    uint16_t sg = tmc_get_stallguard();

    if (sg == 0xFFFFu) {
        printf("  pos: %5ld steps | %6.1f mm | %4.1f%%  SG:UART_FAIL\n",
               (long)motion_get_position(),
               motion_get_position_mm(STEPS_PER_MM),
               motion_get_rail_percent() * 100.0f);
    } else {
        printf("  pos: %5ld steps | %6.1f mm | %4.1f%%  SG:%u\n",
               (long)motion_get_position(),
               motion_get_position_mm(STEPS_PER_MM),
               motion_get_rail_percent() * 100.0f,
               sg);
    }
}

static float clampf_local(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void stop_manual_motion(void) {
    if (g_manual_motion_active || motion_is_moving()) {
        motion_stop_immediate();
    }

    g_manual_direction = 0;
    g_manual_motion_active = false;
}

static void apply_manual_lateral(float lateral) {
    // lateral: -1.0 = full left, +1.0 = full right.
    if (fabsf(lateral) < CONTROLLER_LATERAL_DEADZONE) {
        stop_manual_motion();
        return;
    }

    int direction = (lateral > 0.0f) ? +1 : -1;
    float requested_speed = fabsf(lateral) * g_controller_max_speed;
    requested_speed = clampf_local(requested_speed, 300.0f, g_controller_max_speed);

    int32_t target = (direction > 0) ? mot_get_rail_max() : mot_get_rail_min();

    // Update speed continuously, but do not reissue motion_move_to() every loop.
    // Reissuing motion_move_to() resets acceleration/current_speed in your current TMC library.
    motion_set_acceleration(SAVE_ACCEL, SAVE_DECEL);
    motion_set_speed(requested_speed);

    if (!g_manual_motion_active || direction != g_manual_direction || !motion_is_moving()) {
        motion_move_to(target);
        g_manual_direction = direction;
        g_manual_motion_active = true;
    }
}

static void apply_speed_step(bool increase) {
    if (increase) {
        g_controller_max_speed += CONTROLLER_SPEED_STEP;
        if (g_controller_max_speed > CONTROLLER_MAX_MAX_SPEED) {
            g_controller_max_speed = CONTROLLER_MAX_MAX_SPEED;
        }
        printf("Controller max speed increased: %.0f steps/sec (%.0f mm/sec)\n",
               (double)g_controller_max_speed,
               (double)(g_controller_max_speed / STEPS_PER_MM));
    } else {
        g_controller_max_speed -= CONTROLLER_SPEED_STEP;
        if (g_controller_max_speed < CONTROLLER_MIN_MAX_SPEED) {
            g_controller_max_speed = CONTROLLER_MIN_MAX_SPEED;
        }
        printf("Controller max speed decreased: %.0f steps/sec (%.0f mm/sec)\n",
               (double)g_controller_max_speed,
               (double)(g_controller_max_speed / STEPS_PER_MM));
    }
}

static void robot_control_core1(void) {
    uint32_t last_status_ms = 0;
    bool previous_connected = false;
    bool previous_failsafe = true;

    printf("Robot control loop running on core 1.\n");

    while (true) {
        // Keep the motion ramp and non-blocking solenoid alive.
        motion_update();
        solenoid_update();

        if (stall_flag) {
            if (!g_fault_latched) {
                g_fault_latched = true;
                stop_manual_motion();
                solenoid_force_off();
                rgb_set(true, false, false);
                printf("Fault latched. Reboot or add a controlled re-home state before continuing.\n");
            }
            sleep_ms(5);
            continue;
        }

        xbox::State pad = xbox::XboxController::getState();
        bool failsafe = xbox::XboxController::isFailsafeActive(CONTROLLER_FAILSAFE_MS);

        robo::RobotCommand cmd = robo::RobotCommandMapper::update(pad, failsafe);

        if (pad.connected != previous_connected) {
            previous_connected = pad.connected;
            printf("Xbox controller %s\n", pad.connected ? "connected" : "disconnected");
            rgb_set(false, pad.connected, !pad.connected); // green connected, blue waiting
        }

        if (failsafe != previous_failsafe) {
            previous_failsafe = failsafe;
            if (failsafe) {
                printf("Controller failsafe active: stopping motion.\n");
                stop_manual_motion();
                solenoid_force_off();
            } else {
                printf("Controller failsafe cleared.\n");
            }
        }

        if (cmd.failsafe_active || cmd.emergency_stop) {
            stop_manual_motion();
            if (cmd.emergency_stop) {
                solenoid_force_off();
            }
        } else {
            if (cmd.increase_max_speed) {
                apply_speed_step(true);
            }

            if (cmd.decrease_max_speed) {
                apply_speed_step(false);
            }

            if (cmd.fire_solenoid) {
                printf("RB pressed: solenoid fire.\n");
                solenoid_trigger(KICK_PULSE_MS);
            }

            apply_manual_lateral(cmd.lateral);
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((uint32_t)(now - last_status_ms) >= 500u) {
            last_status_ms = now;

            if (pad.connected && !failsafe) {
                printf("CTRL lateral=%+.2f LT=%.2f RT=%.2f max=%.0f pos=%ld %.1f%%\n",
                       (double)cmd.lateral,
                       (double)pad.lt,
                       (double)pad.rt,
                       (double)g_controller_max_speed,
                       (long)motion_get_position(),
                       (double)(motion_get_rail_percent() * 100.0f));
            } else {
                printf("Waiting for Xbox controller... pos=%ld %.1f%%\n",
                       (long)motion_get_position(),
                       (double)(motion_get_rail_percent() * 100.0f));
            }
        }

        // 1 kHz-ish service loop. The actual STEP pulse generation is still done by
        // the hardware alarm ISR started with motion_timer_start().
        sleep_ms(1);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nSoccer Goalkeeper Controller - MR2004B Pico 2 W PCB\n");
    printf("Debug + Homing + Demo + Xbox Wireless Control\n\n");

    tmc_init();
    solenoid_init();

    rgb_set(false, false, true); // blue = booting
    buzzer_beep(60);
    sleep_ms(100);

    printf("Testing TMC UART...\n");
    tmc_uart_scan_ioin();

    // Configure driver only after confirming the bus is alive.
    // true = StealthChop requested for quieter low-speed testing.
    tmc_configure(IHOLD, IRUN, IHOLDDELAY, MICROSTEPS, true);
    tmc_print_config_status();

    rgb_set(false, true, false); // green = driver configured
    motion_enable(true);

    // StallGuard is intentionally disabled for this quiet/reliable baseline.
    // In StealthChop, SG_RESULT is not a reliable stall detector. Re-enable only
    // after tuning, preferably in SpreadCycle and after checking SG values.
    // motion_set_stall_callback(on_stall);

    // Initial homing.
    if (!do_home()) {
        while (true) tight_loop_contents();
    }

    // Start hardware timer ISR.
    motion_timer_start();

    // During calibrated high-speed moves, use software limits only.
    // The shared physical endstop is used for homing, then disabled to prevent
    // electrical noise from creating false endstop hits during saves/kicks.
    motion_endstop_safety_enable(false);

    // Center.
    printf("Moving to center...\n");
    motion_return_center();
    (void)wait_move();
    print_pos();
    printf("Center move done. Starting demo...\n");
    sleep_ms(500);

    // Demo sequence.
    printf("\n Goalkeeper demo\n");
    printf("SAVE_SPEED = %.0f steps/sec (%.0f mm/sec)\n",
           (double)SAVE_SPEED, (double)(SAVE_SPEED / STEPS_PER_MM));

    printf("SAVE LEFT → KICK\n");
    motion_save_left();
    if (wait_move_checked("SAVE LEFT")) {
        print_pos();
        sleep_ms(100);
        solenoid_fire_blocking(KICK_PULSE_MS, motion_update);
        printf("  Kicked!\n");
    } else {
        printf("  Save left failed, skipping kick.\n");
    }
    sleep_ms(300);

    printf("Return to center\n");
    motion_return_center();
    wait_move_checked("RETURN CENTER 1");
    print_pos();
    sleep_ms(500);

    printf("SAVE RIGHT → KICK\n");
    motion_save_right();
    if (wait_move_checked("SAVE RIGHT")) {
        print_pos();
        sleep_ms(100);
        solenoid_fire_blocking(KICK_PULSE_MS, motion_update);
        printf("  Kicked!\n");
    } else {
        printf("  Save right failed, skipping kick.\n");
    }
    sleep_ms(300);

    printf("Return to center\n");
    motion_return_center();
    wait_move_checked("RETURN CENTER 2");
    print_pos();

    rgb_set(false, true, false); // green = ready
    buzzer_beep(60);
    printf("\nDemo complete. Starting Bluetooth / Xbox controller mode.\n");
    printf("Mapping: RT=right, LT=left, RB=solenoid, D-pad up/down=max speed, B=stop.\n\n");

    // Controller layer setup.
    xbox::XboxController::init();
    xbox::XboxController::setDeadzone(0.10f);
    robo::RobotCommandMapper::init();
    robo::RobotCommandMapper::setTriggerDeadzone(0.03f);

    if (cyw43_arch_init()) {
        printf("CYW43 Wi-Fi/Bluetooth chip init failed.\n");
        rgb_set(true, false, false);
        while (true) tight_loop_contents();
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Bluepad32 custom platform.
    uni_platform_set_custom(xbox_get_bluepad32_platform());
    uni_init(0, NULL);

    // Core 1 keeps the robot responsive while core 0 runs BTstack / Bluepad32.
    multicore_launch_core1(robot_control_core1);

    // Bluetooth run loop. Does not return.
    btstack_run_loop_execute();

    return 0;
}
