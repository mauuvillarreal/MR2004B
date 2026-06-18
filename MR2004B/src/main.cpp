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
#include "vl53l0x.hpp"
#include "external_led.hpp"

#include "hardware/i2c.h"

#define KICK_PULSE_MS   80

#define MPU_I2C_PORT    i2c1
#define MPU_SDA_PIN     2
#define MPU_SCL_PIN     3
#define MPU_I2C_BAUD    100000  

#define ENABLE_I2C_DEVICE_DIAGNOSTICS

#define TOF_I2C_ADDRESS 0x29
static constexpr uint16_t AUTO_KICK_DISTANCE_MM = 180; // Tunear
static constexpr uint16_t AUTO_KICK_RELEASE_MM  = AUTO_KICK_DISTANCE_MM + 40;
static constexpr uint32_t AUTO_KICK_COOLDOWN_MS = 450;
static constexpr uint32_t TOF_SERVICE_PERIOD_MS = 5;
static constexpr uint8_t  TOF_REQUIRED_HITS     = 2;

static constexpr int EXTERNAL_RED_LED_PIN   = 26;
static constexpr int EXTERNAL_GREEN_LED_PIN = 27;

static constexpr bool EXTERNAL_RED_ACTIVE_HIGH   = false;
static constexpr bool EXTERNAL_GREEN_ACTIVE_HIGH = true;

// Speeds in controller mode
static constexpr float CONTROLLER_START_MAX_SPEED   = MANUAL_MAX_SPEED;
static constexpr float CONTROLLER_MIN_MAX_SPEED     = 1000.0f;
static constexpr float CONTROLLER_MAX_MAX_SPEED     = SAVE_SPEED;
static constexpr float CONTROLLER_SPEED_STEP        = 500.0f;
static constexpr float CONTROLLER_LATERAL_DEADZONE  = 0.010f;
static constexpr uint32_t CONTROLLER_FAILSAFE_MS    = 5000;

// Controller behavior.
static constexpr int32_t DPAD_JOG_STEPS             = 80; // 9 mm
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

// ToF / autonomous kick state.
static VL53L0X g_tof(MPU_I2C_PORT, TOF_I2C_ADDRESS);
static bool g_tof_ok = false;
static bool g_auto_kick_enabled = false;
static bool g_auto_kick_latched = false;
static uint8_t g_tof_near_hits = 0;
static uint16_t g_tof_distance_mm = 0;
static bool g_tof_has_sample = false;
static uint32_t g_last_tof_sample_ms = 0;
static uint32_t g_last_tof_service_ms = 0;
static uint32_t g_auto_kick_cooldown_until_ms = 0;

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
static repeating_timer_t g_external_led_service_timer;

static bool external_led_service_callback(repeating_timer_t*) {
    external_led::update();
    return true;
}

static void set_led_status(LedStatus status) {
    if (g_led_status == status) {
        return;
    }

    g_led_status = status;

    const bool external_red =
        (status == LedStatus::DISARMED_CONNECTED) ||
        (status == LedStatus::KICKING);

    const bool external_green = false;

    const bool external_breathe =
        (status == LedStatus::HOMING) ||
        (status == LedStatus::REHOMING) ||
        (status == LedStatus::BT_WAITING) ||
        (status == LedStatus::ARMED_READY) ||
        (status == LedStatus::MANUAL_MOVING);

    external_led::setRed(external_red);
    external_led::setGreen(external_green);
    external_led::setGreenBreathing(external_breathe);

    switch (status) {
        case LedStatus::OFF:
            rgb_off();
            break;

        case LedStatus::BOOTING:
            rgb_set(false, false, true);
            break;

        case LedStatus::DRIVER_READY:
            rgb_set(false, true, false);
            break;

        case LedStatus::HOMING:
            rgb_set(true, true, false);
            break;

        case LedStatus::BT_WAITING:
            rgb_set(false, false, true);
            break;

        case LedStatus::DISARMED_CONNECTED:
            rgb_set(false, true, true);
            break;

        case LedStatus::ARMED_READY:
            rgb_set(false, true, false);
            break;

        case LedStatus::MANUAL_MOVING:
            rgb_set(true, true, true);
            break;

        case LedStatus::AUTO_MOVING:
            rgb_set(true, true, false);
            break;

        case LedStatus::REHOMING:
            rgb_set(true, true, false);
            break;

        case LedStatus::KICKING:
            rgb_set(true, false, true);
            break;

        case LedStatus::FAULT:
            rgb_set(true, false, false);
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

    printf("Homing... (external green LED breathing)\n");
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

#ifdef ENABLE_I2C_DEVICE_DIAGNOSTICS
static bool i2c_address_responds(uint8_t address) {
    if (address < 0x08 || address > 0x77) return false;
    uint8_t byte = 0;
    return i2c_read_timeout_us(MPU_I2C_PORT, address, &byte, 1, false, 2000) >= 0;
}

static void scan_i2c_bus(void) {
    printf("\n========== I2C1 ADDRESS SCAN ==========\n");
    printf("SDA=GP%d | SCL=GP%d | %d Hz\n", MPU_SDA_PIN, MPU_SCL_PIN, MPU_I2C_BAUD);
    printf("Expected: MPU6050 at 0x68 or 0x69, VL53L0X at 0x29\n");

    int found = 0;
    for (uint8_t address = 0x08; address <= 0x77; ++address) {
        if (i2c_address_responds(address)) {
            printf("  Device ACK at 0x%02X", address);
            if (address == 0x29) printf("  <- expected VL53L0X");
            if (address == 0x68 || address == 0x69) printf("  <- expected MPU6050");
            printf("\n");
            ++found;
        }
    }

    if (found == 0) {
        printf("  No I2C devices responded. Check power, common GND, SDA/SCL order, and pull-ups.\n");
    }
    printf("=======================================\n\n");
}
#endif

static void imu_bus_init(void) {
    i2c_init(MPU_I2C_PORT, MPU_I2C_BAUD);
    gpio_set_function(MPU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU_SDA_PIN);
    gpio_pull_up(MPU_SCL_PIN);

#ifdef ENABLE_I2C_DEVICE_DIAGNOSTICS
    scan_i2c_bus();
#endif

    printf("MPU6050 on I2C1 GP%d/GP%d: ", MPU_SDA_PIN, MPU_SCL_PIN);

    g_mpu_ok = g_mpu.init();
    if (g_mpu_ok) {
        printf("FOUND\n");
    } else {
        printf("NOT FOUND - continuing without IMU diagnostics\n");
    }
}

static void tof_init(void) {
    printf("VL53L0X on I2C1 address 0x%02X: ", TOF_I2C_ADDRESS);

    if (!g_tof.testConnection()) {
        printf("NO VALID MODEL RESPONSE - autonomous kick unavailable\n");
        printf("  If the scan shows 0x29, wiring is alive but VL53L0X initialization/register access needs checking.\n");
        printf("  If the scan does not show 0x29, check VIN, GND, SDA, SCL, and XSHUT.\n");
        g_tof_ok = false;
        g_auto_kick_enabled = false;
        return;
    }

    const bool initialized = g_tof.init(true);
    const bool continuous_started = initialized && g_tof.startContinuous();
    g_tof_ok = initialized && continuous_started;

    if (g_tof_ok) {
        printf("FOUND - continuous ranging started\n");
    } else {
        printf("DETECTED, BUT INITIALIZATION FAILED - autonomous kick unavailable\n");
        g_auto_kick_enabled = false;
    }
}
#ifdef ENABLE_I2C_DEVICE_DIAGNOSTICS
static void print_i2c_device_diagnostics(void) {
    // Startup-only checks
    const bool mpu_responding = g_mpu.testConnection();
    const bool tof_responding = g_tof.testConnection();

    printf("\n========== I2C DEVICE DIAGNOSTICS ==========\n");
    printf("Bus: I2C1 | SDA=GP%d | SCL=GP%d | %d Hz\n",
           MPU_SDA_PIN, MPU_SCL_PIN, MPU_I2C_BAUD);
    printf("MPU6050  address 0x68: %s | initialization: %s\n",
           mpu_responding ? "RESPONDING" : "NO RESPONSE",
           g_mpu_ok ? "OK" : "FAILED");
    printf("VL53L0X address 0x%02X: %s | initialization: %s\n",
           TOF_I2C_ADDRESS,
           tof_responding ? "RESPONDING" : "NO RESPONSE",
           g_tof_ok ? "OK" : "FAILED");
    printf("Overall I2C sensor status: %s\n",
           (mpu_responding && g_mpu_ok && tof_responding && g_tof_ok)
               ? "BOTH DEVICES WORKING"
               : "CHECK THE FAILED DEVICE / WIRING");
    printf("============================================\n\n");
}
#endif

static void trigger_kick(const char* source) {
    solenoid_trigger(KICK_PULSE_MS);
    g_kick_led_until_ms = to_ms_since_boot(get_absolute_time()) + 140;
    set_led_status(LedStatus::KICKING);
    printf("Kick: %s\n", source);
}

static void service_tof_auto_kick(void) {
    if (!g_tof_ok) return;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((uint32_t)(now - g_last_tof_service_ms) < TOF_SERVICE_PERIOD_MS) return;
    g_last_tof_service_ms = now;

    uint16_t distance = 0;
    if (!g_tof.readDistanceMm(distance)) return; 
    g_tof_distance_mm = distance;
    g_tof_has_sample = true;
    g_last_tof_sample_ms = now;

    if (distance >= AUTO_KICK_RELEASE_MM) {
        g_auto_kick_latched = false;
        g_tof_near_hits = 0;
        return;
    }

    if (distance == 0 || distance > AUTO_KICK_DISTANCE_MM) {
        g_tof_near_hits = 0;
        return;
    }

    if (g_tof_near_hits < TOF_REQUIRED_HITS) ++g_tof_near_hits;

    const bool cooldown_done = (int32_t)(now - g_auto_kick_cooldown_until_ms) >= 0;
    const bool safe_to_kick = g_auto_kick_enabled && g_robot_armed &&
                              !g_fault_latched && !stall_flag && cooldown_done;

    if (!g_auto_kick_latched && g_tof_near_hits >= TOF_REQUIRED_HITS && safe_to_kick) {
        trigger_kick("VL53L0X autonomous detection");
        g_auto_kick_latched = true;
        g_auto_kick_cooldown_until_ms = now + AUTO_KICK_COOLDOWN_MS;
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

    printf("IMU ax=%+.2fg ay=%+.2fg az=%+.2fg | "
           "gx=%+.1f gy=%+.1f gz=%+.1f dps | "
           "roll=%+.1f pitch=%+.1f | ",
           (double)ax,
           (double)ay,
           (double)az,
           (double)gx,
           (double)gy,
           (double)gz,
           (double)roll_deg,
           (double)pitch_deg);

    if (!g_tof_ok) {
        printf("ToF=OFFLINE | ");
    } else if (!g_tof_has_sample) {
        printf("ToF=NO_DATA | ");
    } else {
        const uint32_t sample_age_ms = now - g_last_tof_sample_ms;

        if (sample_age_ms > 500u) {
            printf("ToF=%u mm STALE(%lu ms) | ",
                   (unsigned)g_tof_distance_mm,
                   (unsigned long)sample_age_ms);
        } else {
            printf("ToF=%u mm | ", (unsigned)g_tof_distance_mm);
        }
    }

    printf("pos=%ld %.1f%%\n",
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
    // Use only for faults/failsafe/emergency behavior.
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
        g_auto_kick_latched = false;
        g_tof_near_hits = 0;
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

    // Return to software limits only.
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
        service_tof_auto_kick();

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

        if (cmd.toggle_auto_kick) {
            g_auto_kick_enabled = !g_auto_kick_enabled && g_tof_ok;
            g_auto_kick_latched = false;
            g_tof_near_hits = 0;
            printf("Autonomous ToF kick %s (threshold=%u mm)\n",
                   g_auto_kick_enabled ? "ENABLED" : "DISABLED",
                   AUTO_KICK_DISTANCE_MM);
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
            trigger_kick("Xbox LB/RB manual command");
        }

        if (g_auto_mode == AutoMode::NONE) {
            // Physical direction is inverted
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
    printf("Debug + Homing + Demo + Xbox Wireless Control + MPU6050 + VL53L0X Auto Kick + RGB/External LEDs\n\n");

    tmc_init();
    solenoid_init();
    external_led::init(EXTERNAL_RED_LED_PIN,
                       EXTERNAL_GREEN_LED_PIN,
                       EXTERNAL_RED_ACTIVE_HIGH,
                       EXTERNAL_GREEN_ACTIVE_HIGH);

#ifdef ENABLE_EXTERNAL_LED_STARTUP_TEST
    printf("External LED test: RED only\n");
    external_led::setGreenBreathing(false);
    external_led::setGreen(false);
    external_led::setRed(true);
    sleep_ms(700);

    printf("External LED test: GREEN only\n");
    external_led::setRed(false);
    external_led::setGreen(true);
    sleep_ms(700);

    printf("External LED test: both OFF\n");
    external_led::setRed(false);
    external_led::setGreen(false);
    sleep_ms(300);
#endif

    if (!add_repeating_timer_ms(-10, external_led_service_callback, nullptr,
                                &g_external_led_service_timer)) {
        printf("WARNING: external LED service timer could not be started.\n");
    }

    imu_bus_init();
    tof_init();

#ifdef ENABLE_I2C_DEVICE_DIAGNOSTICS
    print_i2c_device_diagnostics();
#endif

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
    printf("Mapping: left stick X=FAST movement, RT/LT=slow precision movement, LB=kick, D-pad up/down=speed, D-pad left/right=jog, A=arm, B=disarm, Start=center, hold Back=rehome, Y=toggle autonomous ToF kick.\n\n");

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
