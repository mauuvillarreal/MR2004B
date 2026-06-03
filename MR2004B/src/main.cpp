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
#include "mpu6050.hpp"

#include "hardware/i2c.h"

#define KICK_PULSE_MS   80

#define MPU_I2C_PORT    i2c1
#define MPU_SDA_PIN     2
#define MPU_SCL_PIN     3
#define MPU_I2C_BAUD    400000

// Speeds in controller mode
static constexpr float CONTROLLER_START_MAX_SPEED   = MANUAL_MAX_SPEED;
static constexpr float CONTROLLER_MIN_MAX_SPEED     = 1000.0f;
static constexpr float CONTROLLER_MAX_MAX_SPEED     = SAVE_SPEED;
static constexpr float CONTROLLER_SPEED_STEP        = 500.0f;
static constexpr float CONTROLLER_LATERAL_DEADZONE  = 0.010f;
static constexpr uint32_t CONTROLLER_FAILSAFE_MS    = 5000;

// Controller behavior.
static constexpr int32_t DPAD_JOG_STEPS             = 80; // ~9 mm
static constexpr float MANUAL_TAKEOVER_THRESHOLD    = 0.05f;
static constexpr bool DISARM_DISABLES_DRIVER        = false;

// Manual-control command filtering.
static constexpr float MANUAL_MAIN_MIN_DELTA_STEPS  = 45.0f;
static constexpr float MANUAL_FINE_MIN_DELTA_STEPS  = 12.0f;
static constexpr float MANUAL_MIN_ACTIVE_SPEED      = 450.0f;

// Stall / step-loss recovery.
static volatile bool stall_flag = false;

static float g_controller_max_speed = CONTROLLER_START_MAX_SPEED;
static float g_manual_last_velocity_cmd = 0.0f;
static bool  g_manual_velocity_cmd_valid = false;
static bool  g_manual_stop_already_sent = false;
static int   g_manual_direction = 0;   // -1 = left, +1 = right, 0 = stopped
static bool  g_manual_motion_active = false;
static bool  g_fault_latched = false;
static bool  g_robot_armed = false;

// IMU diagnostic state.
static MPU6050 g_mpu(MPU_I2C_PORT);
static bool g_mpu_ok = false;
static uint32_t g_last_imu_print_ms = 0;
static constexpr uint32_t IMU_PRINT_PERIOD_MS = 250;
static constexpr float RAD_TO_DEG = 57.2957795f;

// LED/status state.
enum class LedStatus {
    OFF,
    BOOTING,
    DRIVER_READY,
    HOMING,
    BT_WAITING,
    DISARMED_CONNECTED,
    ARMED_READY,
    MANUAL_MOVING,
    AUTO_MOVING,
    REHOMING,
    KICKING,
    FAULT,
};

static LedStatus g_led_status = LedStatus::OFF;
static uint32_t g_kick_led_until_ms = 0;

static void set_led_status(LedStatus status) {
    if (g_led_status == status) {
        return;
    }

    g_led_status = status;

    switch (status) {
        case LedStatus::OFF:
            rgb_off();
            break;

        case LedStatus::BOOTING:
            rgb_set(false, false, true); // blue
            break;

        case LedStatus::DRIVER_READY:
            rgb_set(false, true, false); // green
            break;

        case LedStatus::HOMING:
            rgb_set(true, true, false); // yellow
            break;

        case LedStatus::BT_WAITING:
            rgb_set(false, false, true); // blue
            break;

        case LedStatus::DISARMED_CONNECTED:
            rgb_set(false, true, true); // cyan
            break;

        case LedStatus::ARMED_READY:
            rgb_set(false, true, false); // green
            break;

        case LedStatus::MANUAL_MOVING:
            rgb_set(true, true, true); // white
            break;

        case LedStatus::AUTO_MOVING:
            rgb_set(true, true, false); // yellow
            break;

        case LedStatus::REHOMING:
            rgb_set(true, true, false); // yellow
            break;

        case LedStatus::KICKING:
            rgb_set(true, false, true); // magenta
            break;

        case LedStatus::FAULT:
            rgb_set(true, false, false); // red
            break;
    }
}

// Autonomous controller motion state.
enum class AutoMode {
    NONE,
    CENTERING,
    JOGGING,
    REHOMING,
};

static volatile AutoMode g_auto_mode = AutoMode::NONE;

static void on_stall(void) {
    // Called from motion_update() when StallGuard detects a stall.
    stall_flag = true;
    set_led_status(LedStatus::FAULT);
    // buzzer_beep(80);
    printf(" !! STALL DETECTED — will require re-home\n");
}

static bool do_home(void) {
    set_led_status(LedStatus::HOMING);

    printf("Homing...\n");
    bool ok = motion_home(false, HOMING_SPEED, 0);
    if (!ok) {
        set_led_status(LedStatus::FAULT);
        // buzzer_beep(200);
        printf("Homing FAILED — check endstop GP%d\n", ENDSTOP_PIN);
        return false;
    }

    set_led_status(LedStatus::DRIVER_READY);
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

static void print_tmc_driver_snapshot(const char *label) {
    uint32_t gstat = tmc_read(TMC_REG_GSTAT);
    uint32_t drv   = tmc_read(TMC_REG_DRV_STATUS);
    uint32_t ioin  = tmc_read(TMC_REG_IOIN);
    uint32_t ifcnt = tmc_read(TMC_REG_IFCNT);

    printf("\n[TMC SNAPSHOT: %s]\n", label);
    printf("  EN pin: %d  DIR pin: %d  STEP pin: %d  ENDSTOP pin: %d\n",
           gpio_get(EN_PIN),
           gpio_get(DIR_PIN),
           gpio_get(STEP_PIN),
           gpio_get(ENDSTOP_PIN));

    if (gstat == 0xDEADBEEF) printf("  GSTAT:      UART_FAIL\n");
    else                     printf("  GSTAT:      0x%08lX\n", (unsigned long)gstat);

    if (drv == 0xDEADBEEF)   printf("  DRV_STATUS: UART_FAIL\n");
    else                     printf("  DRV_STATUS: 0x%08lX\n", (unsigned long)drv);

    if (ioin == 0xDEADBEEF)  printf("  IOIN:       UART_FAIL\n");
    else                     printf("  IOIN:       0x%08lX\n", (unsigned long)ioin);

    if (ifcnt == 0xDEADBEEF) printf("  IFCNT:      UART_FAIL\n");
    else                     printf("  IFCNT:      0x%08lX\n", (unsigned long)ifcnt);

    printf("  pos=%ld rail_min=%ld rail_max=%ld moving=%d calibrated=%d\n",
           (long)motion_get_position(),
           (long)mot_get_rail_min(),
           (long)mot_get_rail_max(),
           motion_is_moving(),
           motion_is_calibrated());
}

static void imu_bus_init(void) {
    i2c_init(MPU_I2C_PORT, MPU_I2C_BAUD);
    gpio_set_function(MPU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU_SDA_PIN);
    gpio_pull_up(MPU_SCL_PIN);

    printf("MPU6050 on I2C1 GP%d/GP%d: ", MPU_SDA_PIN, MPU_SCL_PIN);

    g_mpu_ok = g_mpu.init();
    if (g_mpu_ok) {
        printf("FOUND\n");
    } else {
        printf("NOT FOUND - continuing without IMU diagnostics\n");
    }
}

static void imu_print_reading(bool force = false) {
    if (!g_mpu_ok) {
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!force && (uint32_t)(now - g_last_imu_print_ms) < IMU_PRINT_PERIOD_MS) {
        return;
    }
    g_last_imu_print_ms = now;

    float ax, ay, az, gx, gy, gz;
    g_mpu.read(ax, ay, az, gx, gy, gz);

    float roll_deg = atan2f(ay, az) * RAD_TO_DEG;
    float pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG;

    printf("IMU ax=%+.2fg ay=%+.2fg az=%+.2fg | gx=%+.1f gy=%+.1f gz=%+.1f dps | roll=%+.1f pitch=%+.1f | pos=%ld %.1f%%\n",
           (double)ax,
           (double)ay,
           (double)az,
           (double)gx,
           (double)gy,
           (double)gz,
           (double)roll_deg,
           (double)pitch_deg,
           (long)motion_get_position(),
           (double)(motion_get_rail_percent() * 100.0f));
}

static float clampf_local(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t clamp_position_to_rail(int32_t pos) {
    if (pos < mot_get_rail_min()) return mot_get_rail_min();
    if (pos > mot_get_rail_max()) return mot_get_rail_max();
    return pos;
}

static void reset_manual_command_cache(void) {
    g_manual_last_velocity_cmd = 0.0f;
    g_manual_velocity_cmd_valid = false;
    g_manual_stop_already_sent = false;
    g_manual_direction = 0;
    g_manual_motion_active = false;
}

static void stop_manual_motion(void) {

    if (!g_manual_stop_already_sent) {
        motion_manual_velocity(0.0f);
        g_manual_stop_already_sent = true;
    }

    g_manual_last_velocity_cmd = 0.0f;
    g_manual_velocity_cmd_valid = false;
    g_manual_direction = 0;
    g_manual_motion_active = false;
}

static void hard_stop_manual_motion(void) {
    // Use this only for faults/failsafe/emergency behavior.
    motion_stop_immediate();
    reset_manual_command_cache();
}

static void apply_manual_lateral(float lateral, bool precision_mode) {
    // lateral: -1.0 = full left, +1.0 = full right.
    if (fabsf(lateral) < CONTROLLER_LATERAL_DEADZONE) {
        stop_manual_motion();
        return;
    }

    float abs_lateral = fabsf(lateral);
    float max_speed = g_controller_max_speed;

    if (precision_mode) {
        max_speed *= robo::RobotCommandMapper::getPrecisionScale();
    }

    if (max_speed < MANUAL_MIN_ACTIVE_SPEED) {
        max_speed = MANUAL_MIN_ACTIVE_SPEED;
    }

    float requested_speed = abs_lateral * max_speed;
    requested_speed = clampf_local(requested_speed, MANUAL_MIN_ACTIVE_SPEED, max_speed);

    float signed_velocity = (lateral > 0.0f) ? requested_speed : -requested_speed;
    float min_delta = precision_mode ? MANUAL_FINE_MIN_DELTA_STEPS : MANUAL_MAIN_MIN_DELTA_STEPS;

    // Event-based manual command
    bool direction_changed = (g_manual_direction != 0) &&
                             ((signed_velocity > 0.0f ? +1 : -1) != g_manual_direction);
    bool velocity_changed = !g_manual_velocity_cmd_valid ||
                            (fabsf(signed_velocity - g_manual_last_velocity_cmd) >= min_delta);

    if (!direction_changed && !velocity_changed && g_manual_motion_active) {
        return;
    }

    motion_set_acceleration(MANUAL_ACCEL, MANUAL_DECEL);
    motion_set_max_speed(g_controller_max_speed);
    motion_manual_velocity(signed_velocity);

    g_manual_last_velocity_cmd = signed_velocity;
    g_manual_velocity_cmd_valid = true;
    g_manual_stop_already_sent = false;
    g_manual_direction = (signed_velocity > 0.0f) ? +1 : -1;
    g_manual_motion_active = true;
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

static bool manual_takeover_requested(const robo::RobotCommand& cmd) {
    return fabsf(cmd.lateral) > MANUAL_TAKEOVER_THRESHOLD;
}

static void set_robot_armed(bool armed) {
    g_robot_armed = armed;

    if (g_robot_armed) {
        motion_enable(true);
        printf("Robot ARMED.\n");
    } else {
        stop_manual_motion();
        solenoid_force_off();
        printf("Robot DISARMED.\n");

        if (DISARM_DISABLES_DRIVER) {
            motion_enable(false);
        }
    }
}

static void start_auto_center(void) {
    if (!motion_is_calibrated()) {
        printf("Return-to-center ignored: rail is not calibrated.\n");
        return;
    }

    printf("Controller command: return to center.\n");
    stop_manual_motion();
    motion_return_center();
    g_auto_mode = AutoMode::CENTERING;
    set_led_status(LedStatus::AUTO_MOVING);
}

static void start_dpad_jog(bool right) {
    if (!motion_is_calibrated()) {
        printf("D-pad jog ignored: rail is not calibrated.\n");
        return;
    }

    int32_t current = motion_get_position();
    int32_t target = current + (right ? DPAD_JOG_STEPS : -DPAD_JOG_STEPS);
    target = clamp_position_to_rail(target);

    printf("Controller command: D-pad %s jog to %ld.\n",
           right ? "right" : "left",
           (long)target);

    stop_manual_motion();
    motion_set_acceleration(RETURN_ACCEL, RETURN_DECEL);
    motion_set_speed(RETURN_SPEED);
    motion_move_to(target);
    g_auto_mode = AutoMode::JOGGING;
    set_led_status(LedStatus::AUTO_MOVING);
}

static void run_rehome_sequence(void) {
    printf("Controller command: re-home requested.\n");

    g_auto_mode = AutoMode::REHOMING;
    set_led_status(LedStatus::REHOMING);

    stop_manual_motion();
    solenoid_force_off();

    // Re-enable the endstop for homing
    motion_endstop_safety_enable(true);

    bool ok = do_home();

    // Return to the normal post-homing policy: software limits only.
    motion_endstop_safety_enable(false);

    if (!ok) {
        printf("Re-home failed. Robot disarmed.\n");
        g_auto_mode = AutoMode::NONE;
        set_robot_armed(false);
        set_led_status(LedStatus::FAULT);
        return;
    }

    printf("Re-home complete. Returning to center.\n");
    motion_return_center();
    g_auto_mode = AutoMode::CENTERING;

    // After a deliberate rehome, keep the robot armed.
    g_robot_armed = true;
    motion_enable(true);
    set_led_status(LedStatus::AUTO_MOVING);
}

static void update_runtime_led(bool connected, bool failsafe, const robo::RobotCommand& cmd) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    bool kick_led_active = (g_kick_led_until_ms != 0) && ((int32_t)(now - g_kick_led_until_ms) < 0);

    if (g_fault_latched || stall_flag) {
        set_led_status(LedStatus::FAULT);
    } else if (kick_led_active) {
        set_led_status(LedStatus::KICKING);
    } else if (!connected || failsafe) {
        set_led_status(LedStatus::BT_WAITING);
    } else if (!g_robot_armed) {
        set_led_status(LedStatus::DISARMED_CONNECTED);
    } else if (g_auto_mode == AutoMode::REHOMING) {
        set_led_status(LedStatus::REHOMING);
    } else if (g_auto_mode != AutoMode::NONE) {
        set_led_status(LedStatus::AUTO_MOVING);
    } else if (fabsf(cmd.lateral) > CONTROLLER_LATERAL_DEADZONE) {
        set_led_status(LedStatus::MANUAL_MOVING);
    } else {
        set_led_status(LedStatus::ARMED_READY);
    }
}

static void robot_control_core1(void) {
    bool previous_connected = false;
    bool previous_failsafe = true;

    printf("Robot control loop running on core 1.\n");

    while (true) {
        motion_update();
        solenoid_update();

        if (stall_flag) {
            if (!g_fault_latched) {
                g_fault_latched = true;
                hard_stop_manual_motion();
                solenoid_force_off();
                g_auto_mode = AutoMode::NONE;
                set_led_status(LedStatus::FAULT);
                printf("Fault latched. Hold View/Back after fixing mechanics, or reboot before continuing.\n");
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
        }

        if (failsafe != previous_failsafe) {
            previous_failsafe = failsafe;
            if (failsafe) {
                printf("Controller failsafe active: stopping motion.\n");
                stop_manual_motion();
                solenoid_force_off();
                g_auto_mode = AutoMode::NONE;
            } else {
                printf("Controller failsafe cleared.\n");
            }
        }

        if (cmd.emergency_stop) {
            printf("Emergency stop / disarm.\n");
            g_auto_mode = AutoMode::NONE;
            set_robot_armed(false);
            update_runtime_led(pad.connected, failsafe, cmd);
            imu_print_reading(false);
            sleep_ms(1);
            continue;
        }

        if (cmd.failsafe_active) {
            stop_manual_motion();
            solenoid_force_off();
            g_auto_mode = AutoMode::NONE;
            update_runtime_led(pad.connected, failsafe, cmd);
            imu_print_reading(false);
            sleep_ms(1);
            continue;
        }

        if (cmd.arm_request) {
            set_robot_armed(true);
        }

        if (cmd.rehome_request && pad.connected && !failsafe) {
            run_rehome_sequence();
        }

        bool takeover = manual_takeover_requested(cmd);

        if (g_auto_mode != AutoMode::NONE) {
            if (takeover) {
                printf("Auto motion cancelled by manual takeover.\n");
                stop_manual_motion();
                g_auto_mode = AutoMode::NONE;
            } else {
                if (!motion_is_moving() && g_auto_mode != AutoMode::REHOMING) {
                    if (g_auto_mode == AutoMode::CENTERING) {
                        printf("Auto centering complete. pos=%ld\n", (long)motion_get_position());
                    } else if (g_auto_mode == AutoMode::JOGGING) {
                        printf("D-pad jog complete. pos=%ld\n", (long)motion_get_position());
                    }
                    g_auto_mode = AutoMode::NONE;
                }

                update_runtime_led(pad.connected, failsafe, cmd);
                imu_print_reading(false);
                sleep_ms(1);
                continue;
            }
        }

        if (!g_robot_armed) {
            stop_manual_motion();
            update_runtime_led(pad.connected, failsafe, cmd);
            imu_print_reading(false);
            sleep_ms(1);
            continue;
        }

        if (cmd.increase_max_speed) {
            apply_speed_step(true);
        }

        if (cmd.decrease_max_speed) {
            apply_speed_step(false);
        }

        if (cmd.return_center) {
            start_auto_center();
        }

        if (cmd.jog_left) {
            start_dpad_jog(false);
        }

        if (cmd.jog_right) {
            start_dpad_jog(true);
        }

        if (cmd.fire_solenoid) {
            solenoid_trigger(KICK_PULSE_MS);
            g_kick_led_until_ms = to_ms_since_boot(get_absolute_time()) + 140;
            set_led_status(LedStatus::KICKING);
        }

        if (g_auto_mode == AutoMode::NONE) {
            // Physical direction is inverted relative to the logical Xbox command.
            apply_manual_lateral(-cmd.lateral, cmd.precision_active);
        }

        update_runtime_led(pad.connected, failsafe, cmd);
        imu_print_reading(false);

        // 1 kHz service loop.
        sleep_ms(1);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nSoccer Goalkeeper Controller - MR2004B Pico 2 W PCB\n");
    printf("Debug + Homing + Demo + Xbox Wireless Control + MPU6050 IMU Diagnostics + RGB Status\n\n");

    tmc_init();
    solenoid_init();
    imu_bus_init();

    set_led_status(LedStatus::BOOTING);
    // buzzer_beep(60);
    sleep_ms(100);

    printf("Testing TMC UART...\n");
    tmc_uart_scan_ioin();

    // false = SpreadCycle. true = StealthChop
    tmc_configure(IHOLD, IRUN, IHOLDDELAY, MICROSTEPS, false);
    tmc_print_config_status();

    set_led_status(LedStatus::DRIVER_READY);
    motion_enable(true);

    // StallGuard is intentionally disabled
    // Re-enable preferably in SpreadCycle.

    // Initial homing.
    if (!do_home()) {
        while (true) tight_loop_contents();
    }

    // Start hardware timer ISR.
    motion_timer_start();

    // In homing use limitswitches, then only software limits.
    motion_endstop_safety_enable(false);

    // Center.
    printf("Moving to center...\n");
    set_led_status(LedStatus::AUTO_MOVING);
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
    set_led_status(LedStatus::AUTO_MOVING);
    motion_save_left();
    if (wait_move_checked("SAVE LEFT")) {
        print_pos();
        print_tmc_driver_snapshot("AFTER SAVE LEFT");
        sleep_ms(100);
        set_led_status(LedStatus::KICKING);
        solenoid_fire_blocking(KICK_PULSE_MS, motion_update);
        printf("  Kicked!\n");
    } else {
        printf("  Save left failed, skipping kick.\n");
    }
    sleep_ms(300);

    printf("Return to center\n");
    set_led_status(LedStatus::AUTO_MOVING);
    motion_return_center();
    wait_move_checked("RETURN CENTER 1");
    print_pos();
    sleep_ms(500);

    printf("SAVE RIGHT → KICK\n");
    set_led_status(LedStatus::AUTO_MOVING);
    motion_save_right();
    if (wait_move_checked("SAVE RIGHT")) {
        print_pos();
        print_tmc_driver_snapshot("AFTER SAVE RIGHT");
        sleep_ms(100);
        set_led_status(LedStatus::KICKING);
        solenoid_fire_blocking(KICK_PULSE_MS, motion_update);
        printf("  Kicked!\n");
    } else {
        printf("  Save right failed, skipping kick.\n");
    }
    sleep_ms(300);

    printf("Return to center\n");
    set_led_status(LedStatus::AUTO_MOVING);
    motion_return_center();
    wait_move_checked("RETURN CENTER 2");
    print_pos();

    set_led_status(LedStatus::DRIVER_READY);
    // buzzer_beep(60);
    printf("\nDemo complete. Starting Bluetooth / Xbox controller mode.\n");
    printf("Mapping: left stick X=FAST movement, RT/LT=slow precision movement, LB=kick, D-pad up/down=speed, D-pad left/right=jog, A=arm, B=disarm, Start=center, hold Back=rehome.\n\n");

    // Controller layer setup.
    xbox::XboxController::init();
    xbox::XboxController::setDeadzone(0.06f);
    robo::RobotCommandMapper::init();
    robo::RobotCommandMapper::setTriggerDeadzone(0.03f);
    robo::RobotCommandMapper::setLeftStickDeadzone(0.08f);
    robo::RobotCommandMapper::setPrecisionScale(0.25f);
    robo::RobotCommandMapper::setRehomeHoldMs(2000);

    if (cyw43_arch_init()) {
        printf("CYW43 Wi-Fi/Bluetooth chip init failed.\n");
        set_led_status(LedStatus::FAULT);
        while (true) tight_loop_contents();
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    set_led_status(LedStatus::BT_WAITING);

    // Bluepad32 custom platform.
    uni_platform_set_custom(xbox_get_bluepad32_platform());
    uni_init(0, NULL);

    // Core 1 keeps the robot responsive while core 0 runs BTstack / Bluepad32.
    multicore_launch_core1(robot_control_core1);

    // Bluetooth run loop. Does not return.
    btstack_run_loop_execute();

    return 0;
}
