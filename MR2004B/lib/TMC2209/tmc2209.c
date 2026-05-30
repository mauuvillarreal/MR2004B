#include "tmc2209.h"
#include "pico/time.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// Hardware timer for step generation
#define STEP_ALARM_NUM  0
static volatile uint32_t isr_step_interval_us = 0;
static volatile bool     isr_alarm_active     = false;
static volatile bool     endstop_safety_enabled = false;
static void reschedule_alarm(uint32_t interval_us);

#define TMC_UART_TIMEOUT_US  5000u
#define TMC_BAD_READ         0xDEADBEEFu

static void uart_drain_rx(void) {
    while (uart_is_readable(UART_ID)) {
        (void)uart_getc(UART_ID);
    }
}

static bool uart_read_byte_timeout(uint8_t *out, uint32_t timeout_us) {
    uint64_t start = time_us_64();
    while (!uart_is_readable(UART_ID)) {
        if ((uint32_t)(time_us_64() - start) >= timeout_us) {
            return false;
        }
        tight_loop_contents();
    }
    *out = uart_getc(UART_ID);
    return true;
}

static bool uart_read_exact_timeout(uint8_t *buf, size_t len, uint32_t timeout_us) {
    for (size_t i = 0; i < len; i++) {
        if (!uart_read_byte_timeout(&buf[i], timeout_us)) {
            return false;
        }
    }
    return true;
}

// Internal State
static volatile MotionState mot = {
    .max_speed = SAVE_SPEED,
    .current_speed = 0.0f,
    .target_speed = 0.0f,
    .manual_velocity_command = 0.0f,
    .manual_velocity_mode = false,
    .acceleration = SAVE_ACCEL,
    .deceleration = SAVE_DECEL,
    .jerk = 0.0f, // S-curve off by default;
    .current_accel = 0.0f,
    .position = 0,
    .target_position = 0,
    .rail_min = 0,
    .rail_max = RAIL_STEPS,
    .rail_calibrated = false,
    .step_interval_us = 0,
    .last_step_time = 0,
    .pid_kp = 1.0f,
    .pid_ki = 0.0f,
    .pid_kd = 0.05f,
    .pid_integral = 0.0f,
    .pid_prev_error = 0.0f,
    .pid_enabled = false,
    .moving = false,
    .direction = true,
    .enabled = false,
    .homing = false,
    .stallguard_threshold = 50,
    .stall_detected = false,
};

static inline float clamp_float(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static inline float approach_float(float current, float target, float max_delta) {
    if (max_delta < 0.0f) max_delta = -max_delta;

    if (target > current + max_delta) {
        return current + max_delta;
    }

    if (target < current - max_delta) {
        return current - max_delta;
    }

    return target;
}

static inline float apply_jerk_limit(float desired_accel, float dt_s) {
    if (mot.jerk <= 0.0f) {
        mot.current_accel = desired_accel;
        return mot.current_accel;
    }

    const float max_accel_delta = mot.jerk * dt_s;
    mot.current_accel = approach_float(mot.current_accel, desired_accel, max_accel_delta);
    return mot.current_accel;
}

static inline void stop_motion_state(bool clear_manual_mode) {
    if (clear_manual_mode) {
        mot.manual_velocity_mode = false;
        mot.manual_velocity_command = 0.0f;
    }

    mot.target_position = mot.position;
    mot.current_speed = 0.0f;
    mot.target_speed = 0.0f;
    mot.current_accel = 0.0f;
    mot.moving = false;
    isr_step_interval_us = 0;
    isr_alarm_active = false;
}


// CRC
uint8_t tmc_crc(uint8_t *data, uint8_t length) {
    uint8_t crc = 0;

    for (uint8_t i = 0; i < length; i++) {
        uint8_t current_byte = data[i];

        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }

            current_byte >>= 1;
        }
    }

    return crc;
}

// UART TMC Write / Read
void tmc_write(uint8_t reg, uint32_t value) {
    uint8_t packet[8];
    packet[0] = 0x05;
    packet[1] = 0x00;
    packet[2] = reg | 0x80;
    packet[3] = (value >> 24) & 0xFF;
    packet[4] = (value >> 16) & 0xFF;
    packet[5] = (value >> 8)  & 0xFF;
    packet[6] =  value        & 0xFF;
    packet[7] = tmc_crc(packet, 7);

    uart_drain_rx();
    uart_write_blocking(UART_ID, packet, 8);
    uart_tx_wait_blocking(UART_ID);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(UART_TX_PIN, GPIO_IN);
    gpio_pull_up(UART_TX_PIN);
    busy_wait_us_32(20);

    // Read and discard echo, bail as soon as we have 8 bytes
    uint8_t dummy[8];
    (void)uart_read_exact_timeout(dummy, 8, TMC_UART_TIMEOUT_US);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
}

uint32_t tmc_read(uint8_t reg) {
    uint8_t req[4];
    req[0] = 0x05;
    req[1] = 0x00;
    req[2] = reg & 0x7F;
    req[3] = tmc_crc(req, 3);

    uart_drain_rx();
    uart_write_blocking(UART_ID, req, 4);
    uart_tx_wait_blocking(UART_ID);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(UART_TX_PIN, GPIO_IN);
    gpio_pull_up(UART_TX_PIN);
    busy_wait_us_32(20);

    // Read echo + reply (12 bytes)
    uint8_t rx[16];
    size_t n = 0;
    uint64_t start = time_us_64();

    while (n < sizeof(rx) && (uint32_t)(time_us_64() - start) < TMC_UART_TIMEOUT_US) {
        if (uart_is_readable(UART_ID)) {
            rx[n++] = uart_getc(UART_ID);
        } else {
            tight_loop_contents();
        }
    }

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    if (n < 8) return TMC_BAD_READ;

    // Search specifically for a TMC read reply frame
    for (size_t off = 0; off + 7 < n; off++) {
        if (rx[off] != 0x05) continue;
        if (rx[off + 1] != 0xFF) continue;
        if (rx[off + 2] != (reg & 0x7F)) continue;

        uint8_t crc = tmc_crc(&rx[off], 7);
        if (crc == rx[off + 7]) {
            return ((uint32_t)rx[off + 3] << 24) |
                   ((uint32_t)rx[off + 4] << 16) |
                   ((uint32_t)rx[off + 5] <<  8) |
                    (uint32_t)rx[off + 6];
        }
    }

    return TMC_BAD_READ;
}

// Microstep resolution bits (MRES field in CHOPCONF)
uint32_t tmc_microstep_bits(uint16_t microsteps) {
    // MRES: 0=256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2, 8=1
    switch (microsteps) {
        case 1:   return 8;
        case 2:   return 7;
        case 4:   return 6;
        case 8:   return 5;
        case 16:  return 4;
        case 32:  return 3;
        case 64:  return 2;
        case 128: return 1;
        case 256: return 0;
        default:  return 4; // 16 microsteps default
    }
}

// Initialisation
static inline bool active_level(bool value, bool active_high) {
    return active_high ? value : !value;
}

void rgb_set(bool r, bool g, bool b) {
    gpio_put(RGB_R_PIN, active_level(r, RGB_ACTIVE_HIGH));
    gpio_put(RGB_G_PIN, active_level(g, RGB_ACTIVE_HIGH));
    gpio_put(RGB_B_PIN, active_level(b, RGB_ACTIVE_HIGH));
}

void rgb_off(void) {
    rgb_set(false, false, false);
}

void buzzer_set(bool on) {
    gpio_put(BUZZER_PIN, active_level(on, BUZZER_ACTIVE_HIGH));
}

void buzzer_beep(uint32_t duration_ms) {
    buzzer_set(true);
    sleep_ms(duration_ms);
    buzzer_set(false);
}

bool limit_switch_pressed(void) {
    return !gpio_get(ENDSTOP_PIN); // active LOW
}

void tmc_init(void) {
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, HOME_DIR);

    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 1); // disabled at boot, TMC2209 EN is active LOW

    gpio_init(RGB_R_PIN);
    gpio_set_dir(RGB_R_PIN, GPIO_OUT);
    gpio_init(RGB_G_PIN);
    gpio_set_dir(RGB_G_PIN, GPIO_OUT);
    gpio_init(RGB_B_PIN);
    gpio_set_dir(RGB_B_PIN, GPIO_OUT);
    rgb_off();

    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    buzzer_set(false);

    gpio_init(ENDSTOP_PIN);
    gpio_set_dir(ENDSTOP_PIN, GPIO_IN);
    gpio_pull_up(ENDSTOP_PIN);

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(UART_RX_PIN);
    gpio_pull_up(UART_TX_PIN);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    sleep_ms(200);
}

static uint8_t tmc_ifcnt_value(void) {
    uint32_t v = tmc_read(TMC_REG_IFCNT);
    if (v == TMC_BAD_READ) return 0xFFu;
    return (uint8_t)(v & 0xFFu);
}

static bool tmc_write_counted(uint8_t reg, uint32_t value, const char *name) {
    uint8_t before = tmc_ifcnt_value();
    tmc_write(reg, value);
    sleep_ms(8);
    uint8_t after = tmc_ifcnt_value();

    if (before == 0xFFu || after == 0xFFu) {
        printf("%-12s write 0x%08lX | IFCNT read failed\n", name, value);
        return false;
    }

    uint8_t delta = (uint8_t)(after - before); // wraps naturally at 255
    printf("%-12s write 0x%08lX | IFCNT %u -> %u | %s\n",
           name, value, before, after, delta ? "accepted" : "NO COUNT");

    return delta != 0;
}

void tmc_configure(uint8_t ihold, uint8_t irun, uint8_t iholddelay,
                   uint8_t microsteps, bool stealthchop) {
    printf("\nConfiguring TMC2209...\n");

    // Put driver into UART-controlled mode first.
    uint32_t gconf = (1u << 6) | (1u << 7);

    // en_SpreadCycle = 1 means SpreadCycle.
    // en_SpreadCycle = 0 means StealthChop.
    if (!stealthchop) {
        gconf |= (1u << 2);
    }
    tmc_write_counted(TMC_REG_GCONF, gconf, "GCONF");

    // Current control. IHOLD_IRUN is usually write-only, so we verify with IFCNT.
    uint32_t ihold_irun =
        ((uint32_t)ihold) |
        ((uint32_t)irun << 8) |
        ((uint32_t)iholddelay << 16);
    printf("Expected IHOLD_IRUN fields: IHOLD=%u IRUN=%u IHOLDDELAY=%u\n",
           ihold, irun, iholddelay);
    tmc_write_counted(TMC_REG_IHOLD_IRUN, ihold_irun, "IHOLD_IRUN");

    // 3) Chopper config. CHOPCONF is readable, so we also print readback later.
    // MRES bits [27:24]
    // INTPOL bit 28 = interpolate to 256 microsteps
    uint32_t mres = tmc_microstep_bits(microsteps);
    uint32_t chopconf =
        (1u << 28) | // INTPOL = 1
        (mres << 24) |
        (2u << 15)  | // TBL
        (3u << 0)   | // TOFF
        (5u << 4)   | // HSTRT
        (3u << 7); // HEND
    tmc_write_counted(TMC_REG_CHOPCONF, chopconf, "CHOPCONF");

    // StealthChop threshold. TPWMTHRS is commonly write-only; verify with IFCNT.
    tmc_write_counted(TMC_REG_TPWMTHRS, 500u, "TPWMTHRS");

    // PWM config. PWMCONF is readable.
    tmc_write_counted(TMC_REG_PWMCONF, 0xC10D0024u, "PWMCONF");

    // StallGuard threshold, only useful later in SpreadCycle tuning.
    tmc_write_counted(TMC_REG_SGTHRS, mot.stallguard_threshold, "SGTHRS");

    // Powerdown delay. Also usually write-only; verify with IFCNT.
    tmc_write_counted(TMC_REG_TPOWERDOWN, 20u, "TPOWERDOWN");

    sleep_ms(20);
}

void tmc_print_config_status(void) {
    uint32_t gconf = tmc_read(TMC_REG_GCONF);
    uint32_t gstat = tmc_read(TMC_REG_GSTAT);
    uint32_t ifcnt = tmc_read(TMC_REG_IFCNT);
    uint32_t chop  = tmc_read(TMC_REG_CHOPCONF);
    uint32_t pwm   = tmc_read(TMC_REG_PWMCONF);
    uint32_t ioin  = tmc_read(TMC_REG_IOIN);

    printf("\nTMC2209 status/readback:\n");
    printf("  IOIN:     0x%08lX\n", ioin);
    printf("  GCONF:    0x%08lX\n", gconf);
    printf("  GSTAT:    0x%08lX  (reset/uv_cp flags if nonzero)\n", gstat);
    printf("  IFCNT:    0x%08lX\n", ifcnt);
    printf("  CHOPCONF: 0x%08lX  MRES=%lu  INTPOL=%lu\n",
           chop, (chop >> 24) & 0x0Fu, (chop >> 28) & 0x01u);
    printf("  PWMCONF:  0x%08lX\n", pwm);
    printf("  IHOLD_IRUN expected from firmware: IHOLD=%u IRUN=%u IHOLDDELAY=%u\n",
           IHOLD, IRUN, IHOLDDELAY);
    printf("  TPWMTHRS expected from firmware: 500 (0x000001F4)\n");
    printf("  Note: IHOLD_IRUN and TPWMTHRS may read as 0 because they are write-only/limited-readback on many TMC2209 modules. IFCNT is the write verification.\n\n");
}

// Motion Enable
void motion_enable(bool en) {
    mot.enabled = en;
    gpio_put(EN_PIN, en ? 0 : 1); // EN is active LOW
    if (!en) {
        mot.manual_velocity_mode = false;
        mot.manual_velocity_command = 0.0f;
        mot.moving = false;
        mot.current_speed = 0.0f;
    }
}

void motion_endstop_safety_enable(bool en) {
    endstop_safety_enabled = en;
}

// Speed & Acceleration
void motion_set_speed(float steps_per_sec) {
    if (steps_per_sec < 0) steps_per_sec = 0;
    if (steps_per_sec > mot.max_speed) steps_per_sec = mot.max_speed;
    mot.target_speed = steps_per_sec;
}

void motion_set_max_speed(float steps_per_sec) {
    if (steps_per_sec < 0.0f) steps_per_sec = 0.0f;
    mot.max_speed = steps_per_sec;

    if (mot.target_speed > mot.max_speed) {
        mot.target_speed = mot.max_speed;
    }

    if (fabsf(mot.manual_velocity_command) > mot.max_speed) {
        mot.manual_velocity_command = mot.manual_velocity_command >= 0.0f ? mot.max_speed : -mot.max_speed;
    }
}

float motion_get_max_speed(void) {
    return mot.max_speed;
}

void motion_set_acceleration(float accel, float decel) {
    mot.acceleration = (accel > 0) ? accel : 100.0f;
    mot.deceleration = (decel > 0) ? decel : 100.0f;
}

// Move Commands
void motion_move_to(int32_t target_steps) {
    if (mot.rail_calibrated) {
        if (target_steps < mot.rail_min) target_steps = mot.rail_min;
        if (target_steps > mot.rail_max) target_steps = mot.rail_max;
    }
    uint32_t irq = save_and_disable_interrupts();

    mot.manual_velocity_mode = false;
    mot.manual_velocity_command = 0.0f;
    mot.target_position = target_steps;
    mot.pid_integral = 0.0f;
    mot.pid_prev_error = 0.0f;
    mot.stall_detected = false;

    if (target_steps == mot.position) {
        mot.moving = false;
        mot.current_speed = 0.0f;
        mot.current_accel = 0.0f;
        isr_step_interval_us = 0;
        isr_alarm_active = false;
        restore_interrupts(irq);
        return;
    }

    mot.direction = (target_steps > mot.position);
    mot.current_speed = 0.0f;
    mot.current_accel = 0.0f;
    mot.moving = true;

    restore_interrupts(irq);

    // Set DIR before arming the ISR.
    gpio_put(DIR_PIN, mot.direction ? 1 : 0);
    busy_wait_us_32(10);

    // Start the hardware alarm.
    uint32_t start_interval = (uint32_t)(1000000.0f / 500.0f);
    isr_step_interval_us = start_interval;
    reschedule_alarm(start_interval);
}

void motion_move_relative(int32_t delta_steps) {
    motion_move_to(mot.position + delta_steps);
}

void motion_manual_velocity(float velocity_steps_per_sec) {
    if (!mot.enabled) {
        return;
    }

    const float stop_deadband = 1.0f;

    if (velocity_steps_per_sec > MANUAL_MAX_SPEED) {
        velocity_steps_per_sec = MANUAL_MAX_SPEED;
    } else if (velocity_steps_per_sec < -MANUAL_MAX_SPEED) {
        velocity_steps_per_sec = -MANUAL_MAX_SPEED;
    }

    // If the same manual command is already active, do nothing. The motion
    // profile will continue running by itself. Avoiding repeated 1 kHz command
    // writes is important because they briefly disable interrupts and can add
    // manual-only step jitter.
    {
        uint32_t irq = save_and_disable_interrupts();
        bool duplicate = mot.manual_velocity_mode &&
                         fabsf(mot.manual_velocity_command - velocity_steps_per_sec) < 0.5f &&
                         mot.moving;
        restore_interrupts(irq);
        if (duplicate) return;
    }

    // Command released: use S-curve deceleration down to zero, but keep EN active
    // so the motor continues holding torque.
    if (fabsf(velocity_steps_per_sec) <= stop_deadband) {
        uint32_t irq = save_and_disable_interrupts();
        mot.manual_velocity_mode = true;
        mot.manual_velocity_command = 0.0f;
        mot.target_speed = 0.0f;
        mot.max_speed = MANUAL_MAX_SPEED;
        mot.acceleration = MANUAL_ACCEL;
        mot.deceleration = MANUAL_DECEL;
        mot.jerk = MANUAL_JERK;
        if (mot.current_speed > 1.0f) {
            mot.moving = true;
        }
        restore_interrupts(irq);
        return;
    }

    bool new_direction = (velocity_steps_per_sec > 0.0f);
    int32_t new_target;

    if (mot.rail_calibrated) {
        new_target = new_direction ? mot.rail_max : mot.rail_min;

        // Do not push harder into a calibrated rail limit. This stops stepping
        // but does not disable the driver.
        if ((new_direction && mot.position >= mot.rail_max) ||
            (!new_direction && mot.position <= mot.rail_min)) {
            motion_stop_immediate();
            return;
        }
    } else {
        new_target = mot.position + (new_direction ? 1000000 : -1000000);
    }

    uint32_t irq = save_and_disable_interrupts();

    // Direction changes are handled by motion_update() by ramping signed speed
    // through zero. Do NOT flip DIR here while current_speed is still nonzero.
    bool direction_changed = mot.moving && (mot.direction != new_direction) && (mot.current_speed > 1.0f);
    bool needs_start = !mot.moving || !mot.manual_velocity_mode;

    mot.manual_velocity_mode = true;
    mot.manual_velocity_command = velocity_steps_per_sec;
    mot.target_speed = fabsf(velocity_steps_per_sec);
    mot.max_speed = MANUAL_MAX_SPEED;
    mot.acceleration = MANUAL_ACCEL;
    mot.deceleration = MANUAL_DECEL;
    mot.jerk = MANUAL_JERK;
    mot.target_position = new_target;
    mot.pid_enabled = false;
    mot.pid_integral = 0.0f;
    mot.pid_prev_error = 0.0f;
    mot.stall_detected = false;

    if (needs_start) {
        mot.direction = new_direction;
        mot.moving = true;
    } else if (!direction_changed && mot.current_speed <= 1.0f) {
        mot.direction = new_direction;
    }

    restore_interrupts(irq);

    // Only write DIR immediately when stopped/starting or continuing same direction.
    // During reversals, motion_update() changes DIR only after speed crosses zero.
    if (!direction_changed) {
        gpio_put(DIR_PIN, mot.direction ? 1 : 0);
        busy_wait_us_32(10);
    }

    if (needs_start || !isr_alarm_active) {
        uint32_t start_interval = (uint32_t)(1000000.0f / 500.0f);
        isr_step_interval_us = start_interval;
        reschedule_alarm(start_interval);
    }
}


void motion_manual_stop(void) {
    // Teleop release should stop step generation immediately, but it should NOT
    // disable the driver. EN remains low, so the NEMA should keep holding torque.
    motion_stop_immediate();
}

bool motion_is_manual_velocity_mode(void) {
    return mot.manual_velocity_mode;
}

void motion_stop(void) {
    mot.manual_velocity_mode = false;
    mot.manual_velocity_command = 0.0f;
    mot.target_position = mot.position; // decelerate to stop here
    mot.target_speed    = 0.0f;
}

void motion_stop_immediate(void) {
    stop_motion_state(true);
}


void motion_set_jerk(float jerk) {
    mot.jerk = (jerk > 0.0f) ? jerk : 0.0f;
}

float motion_get_jerk(void) {
    return mot.jerk;
}

void motion_move_to_mm(float mm) {
    motion_move_to((int32_t)(mm * STEPS_PER_MM));
}

// Goalkeeper helpers
void motion_save(float target_mm) {
    motion_set_max_speed(SAVE_SPEED);
    motion_set_speed(SAVE_SPEED);
    motion_set_acceleration(SAVE_ACCEL, SAVE_DECEL);
    motion_set_jerk(SAVE_JERK);

    // Convert mm to steps relative to calibrated rail_min.
    // This function is useful for future ball-position targeting.
    // For hard left/right demo saves, use motion_save_left/right so the
    // right side is always the measured rail_max, not an old RAIL_LENGTH_MM.
    int32_t target = mot.rail_min + (int32_t)(target_mm * STEPS_PER_MM);
    motion_move_to(target);
}

void motion_save_left(void) {
    motion_set_max_speed(SAVE_SPEED);
    motion_set_speed(SAVE_SPEED);
    motion_set_acceleration(SAVE_ACCEL, SAVE_DECEL);
    motion_set_jerk(SAVE_JERK);
    motion_move_to(mot.rail_min);
}

void motion_save_right(void) {
    motion_set_max_speed(SAVE_SPEED);
    motion_set_speed(SAVE_SPEED);
    motion_set_acceleration(SAVE_ACCEL, SAVE_DECEL);
    motion_set_jerk(SAVE_JERK);
    motion_move_to(mot.rail_max);
}

void motion_return_center(void) {
    motion_set_max_speed(RETURN_SPEED);
    motion_set_speed(RETURN_SPEED);
    motion_set_acceleration(RETURN_ACCEL, RETURN_DECEL);
    motion_set_jerk(RETURN_JERK);
    int32_t center = mot.rail_min + (mot.rail_max - mot.rail_min) / 2;
    motion_move_to(center);
}


// PID Configuration
void motion_pid_configure(float kp, float ki, float kd) {
    mot.pid_kp = kp;
    mot.pid_ki = ki;
    mot.pid_kd = kd;
    mot.pid_integral   = 0.0f;
    mot.pid_prev_error = 0.0f;
}

void motion_pid_enable(bool en) {
    mot.pid_enabled = en;
    mot.pid_integral   = 0.0f;
    mot.pid_prev_error = 0.0f;
}

float motion_pid_compute(int32_t setpoint, int32_t measurement, float dt) {
    float error = (float)(setpoint - measurement);
    mot.pid_integral += error * dt;
    float derivative = (dt > 0.0f) ? (error - mot.pid_prev_error) / dt : 0.0f;
    mot.pid_prev_error = error;

    // Anti-windup: clamp integral
    float integral_limit = mot.max_speed / (mot.pid_ki > 0 ? mot.pid_ki : 1.0f);
    if (mot.pid_integral >  integral_limit) mot.pid_integral =  integral_limit;
    if (mot.pid_integral < -integral_limit) mot.pid_integral = -integral_limit;

    float output = mot.pid_kp * error +
                   mot.pid_ki * mot.pid_integral +
                   mot.pid_kd * derivative;
    return output;
}

// Debounce helper
static bool endstop_debounced(uint pin, uint8_t count, uint32_t delay_us) {
    uint8_t confirmed = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (!gpio_get(pin)) confirmed++;  // LOW = triggered (NC switch pressed)
        else confirmed = 0;
        sleep_us(delay_us);
    }
    return confirmed >= count;
}

// Acceleration Ramp
static float braking_distance(float speed, float decel) {
    if (decel <= 0 || speed <= 0) return 0.0f;
    return (speed * speed) / (2.0f * decel);
}

// Main Update — call in tight loop or timer ISR
void motion_update(void) {
    static uint64_t last_update_time = 0;

    if (!mot.enabled || !mot.moving) {
        mot.current_speed = 0.0f;
        mot.current_accel = 0.0f;
        last_update_time = time_us_64();
        return;
    }

    uint64_t now = time_us_64();
    float dt_ramp = (last_update_time > 0) ? (float)(now - last_update_time) * 1e-6f : 0.001f;
    if (dt_ramp <= 0.0f || dt_ramp > 0.05f) {
        dt_ramp = 0.001f;
    }
    last_update_time = now;

    int32_t remaining = mot.target_position - mot.position;

    if (remaining == 0) {
        stop_motion_state(true);
        return;
    }

    // Manual velocity mode
    if (mot.manual_velocity_mode) {
        float target_signed_speed = mot.manual_velocity_command;

        if (mot.rail_calibrated) {
            if ((mot.position >= mot.rail_max && target_signed_speed > 0.0f) ||
                (mot.position <= mot.rail_min && target_signed_speed < 0.0f)) {
                target_signed_speed = 0.0f;
            }
        }

        target_signed_speed = clamp_float(target_signed_speed, -mot.max_speed, mot.max_speed);

        float current_signed_speed = mot.direction ? mot.current_speed : -mot.current_speed;
        float speed_error = target_signed_speed - current_signed_speed;

        float desired_accel = 0.0f;
        if (fabsf(speed_error) > 0.5f) {
            float same_sign = current_signed_speed * target_signed_speed;
            bool speeding_up_same_direction =
                (same_sign >= 0.0f) && (fabsf(target_signed_speed) > fabsf(current_signed_speed));

            float accel_limit = speeding_up_same_direction ? mot.acceleration : mot.deceleration;
            desired_accel = clamp_float(speed_error / dt_ramp, -accel_limit, accel_limit);
        }

        float accel_now = apply_jerk_limit(desired_accel, dt_ramp);

        if (fabsf(speed_error) <= fabsf(accel_now * dt_ramp)) {
            current_signed_speed = target_signed_speed;
            mot.current_accel = 0.0f;
        } else {
            current_signed_speed += accel_now * dt_ramp;
        }

        if (fabsf(target_signed_speed) < 1.0f && fabsf(current_signed_speed) < 1.0f) {
            stop_motion_state(true);
            return;
        }

        mot.direction = (current_signed_speed >= 0.0f);
        mot.current_speed = fabsf(current_signed_speed);

        if (mot.current_speed > mot.max_speed) {
            mot.current_speed = mot.max_speed;
        }

        gpio_put(DIR_PIN, mot.direction ? 1 : 0);

        if (mot.current_speed < 1.0f) {
            isr_step_interval_us = 0;
        } else {
            isr_step_interval_us = (uint32_t)(1000000.0f / mot.current_speed);
            if (!isr_alarm_active) {
                reschedule_alarm(isr_step_interval_us);
            }
        }
    } else {
        float abs_remaining = fabsf((float)remaining);

        // PID mode: override target_speed with PID output
        if (mot.pid_enabled) {
            static uint64_t last_pid_time = 0;
            float dt = (last_pid_time > 0) ? (float)(now - last_pid_time) * 1e-6f : 0.001f;
            last_pid_time = now;
            float pid_out = motion_pid_compute(mot.target_position, mot.position, dt);
            mot.target_speed = fabsf(pid_out);
            if (mot.target_speed > mot.max_speed) mot.target_speed = mot.max_speed;
        }

        float bd = braking_distance(mot.current_speed, mot.deceleration);
        float desired_speed;

        if (abs_remaining <= bd + 1.0f) {
            desired_speed = sqrtf(2.0f * mot.deceleration * abs_remaining);
            if (desired_speed < 50.0f) desired_speed = 50.0f;
        } else {
            desired_speed = mot.target_speed > 0.0f ? mot.target_speed : mot.max_speed;
        }

        desired_speed = clamp_float(desired_speed, 0.0f, mot.max_speed);

        float speed_error = desired_speed - mot.current_speed;
        float desired_accel = 0.0f;

        if (fabsf(speed_error) > 0.5f) {
            float accel_limit = (speed_error > 0.0f) ? mot.acceleration : mot.deceleration;
            desired_accel = clamp_float(speed_error / dt_ramp, -accel_limit, accel_limit);
        }

        float accel_now = apply_jerk_limit(desired_accel, dt_ramp);

        if (fabsf(speed_error) <= fabsf(accel_now * dt_ramp)) {
            mot.current_speed = desired_speed;
            mot.current_accel = 0.0f;
        } else {
            mot.current_speed += accel_now * dt_ramp;
        }

        mot.current_speed = clamp_float(mot.current_speed, 0.0f, mot.max_speed);

        if (mot.current_speed < 1.0f) {
            isr_step_interval_us = 0;
        } else {
            isr_step_interval_us = (uint32_t)(1000000.0f / mot.current_speed);
            if (!isr_alarm_active) {
                reschedule_alarm(isr_step_interval_us);
            }
        }
    }

    // Optional physical endstop safety. Keep this OFF during normal demo/save moves.
    if (endstop_safety_enabled && mot.moving && endstop_debounced(ENDSTOP_PIN, 8, 250)) {
        mot.position = mot.direction ? mot.rail_max : mot.rail_min;
        motion_stop_immediate();
        isr_step_interval_us = 0;
    }

    // StallGuard stall detection.
    static bool was_moving = false;
    static uint32_t stall_count = 0;
    static uint64_t move_start_us = 0;

    if (mot.moving && !was_moving) {
        move_start_us = now;
        stall_count = 0;
    }
    was_moving = mot.moving;

    if (mot.moving && mot.current_speed > 8000.0f && mot.stall_callback != NULL &&
        (now - move_start_us) > 30000) {
        uint16_t sg = tmc_get_stallguard();
        if (sg == 0) {
            stall_count++;
            if (stall_count >= mot.stall_debounce) {
                void (*cb)(void) = mot.stall_callback;
                stall_count = 0;
                mot.stall_detected = true;
                motion_stop_immediate();
                if (cb != NULL) cb();
            }
        } else if (sg != 0xFFFFu) {
            stall_count = 0;
        }
    }
}

static void reschedule_alarm(uint32_t interval_us) {
    if (interval_us < 10) interval_us = 10; // hard floor: 100 000 steps/sec max
    isr_alarm_active = true;
    hardware_alarm_set_target(STEP_ALARM_NUM, make_timeout_time_us(interval_us));
}

static void step_alarm_isr(uint alarm_num) {
    (void)alarm_num;
    // Interrupt is automatically cleared when the callback fires.

    isr_alarm_active = false;

    if (!mot.moving || !mot.enabled || isr_step_interval_us == 0) return;

    // Step pulse
    gpio_put(STEP_PIN, 1);
    busy_wait_us_32(2);
    gpio_put(STEP_PIN, 0);

    mot.position += mot.direction ? 1 : -1;

    // Check arrival
    if (mot.position == mot.target_position) {
        mot.manual_velocity_mode = false;
        mot.manual_velocity_command = 0.0f;
        mot.moving = false;
        mot.current_speed = 0.0f;
        isr_step_interval_us = 0;
        isr_alarm_active = false;
        return;
    }

    // Optional physical endstop safety. Disabled during normal goalkeeper moves
    // to avoid false stops caused by electrical noise on the shared endstop pin.
    if (endstop_safety_enabled && !gpio_get(ENDSTOP_PIN)) {
        mot.position      = mot.direction ? mot.rail_max : mot.rail_min;
        mot.moving        = false;
        mot.current_speed = 0.0f;
        isr_step_interval_us = 0;
        isr_alarm_active = false;
        return;
    }

    // Reschedule at updated interval
    reschedule_alarm(isr_step_interval_us);
}

void motion_timer_start(void) {
    hardware_alarm_claim(STEP_ALARM_NUM);
    hardware_alarm_set_callback(STEP_ALARM_NUM, step_alarm_isr);
}

// StallGuard
uint16_t tmc_get_stallguard(void) {
    uint32_t sg = tmc_read(TMC_REG_SG_RESULT);
    if (sg == TMC_BAD_READ) {
        return 0xFFFFu; // invalid/no UART reply; caller must not treat this as a stall
    }
    return (uint16_t)(sg & 0x3FF); // 10-bit value
}

bool tmc_check_stall(void) {
    uint16_t sg = tmc_get_stallguard();
    mot.stall_detected = (sg == 0);
    return mot.stall_detected;
}

void motion_set_stall_callback(void (*cb)(void)) {
    mot.stall_callback   = cb;
    mot.stall_debounce   = 3; // require 3 consecutive zero SG readings
    mot.stall_detected   = false;
}


static uint8_t amps_to_cs(float amps) {
    if (amps < 0.0f) amps = 0.0f;
    if (amps > 2.0f) amps = 2.0f; // hard cap
    uint8_t cs = (uint8_t)(amps / 2.0f * 31.0f);
    if (cs > 31) cs = 31;
    return cs;
}

static float cs_to_amps(uint8_t cs) {
    return (float)cs / 31.0f * 2.0f;
}

void tmc_set_current(float irun_amps, float ihold_amps) {
    uint8_t irun  = amps_to_cs(irun_amps);
    uint8_t ihold = amps_to_cs(ihold_amps);
    uint32_t val  = ((uint32_t)ihold) |
                    ((uint32_t)irun  << 8) |
                    ((uint32_t)IHOLDDELAY << 16);
    tmc_write(TMC_REG_IHOLD_IRUN, val);
    printf("  Current set: IRUN=%.2fA (cs=%d)  IHOLD=%.2fA (cs=%d)\n",
           cs_to_amps(irun), irun, cs_to_amps(ihold), ihold);
}

float tmc_get_current_scale(void) {
    uint32_t pwm_scale = tmc_read(TMC_REG_PWM_SCALE);
    if (pwm_scale == TMC_BAD_READ) return -1.0f;
    uint8_t  scale_sum = pwm_scale & 0xFF;  // bits [7:0]
    return (float)scale_sum;
}

void tmc_print_status(void) {
    uint32_t pwm_scale = tmc_read(TMC_REG_PWM_SCALE);
    uint16_t sg        = tmc_get_stallguard();

    if (pwm_scale == TMC_BAD_READ || sg == 0xFFFFu) {
        printf("  TMC status | UART read failed | speed: %.0f steps/s\n", mot.current_speed);
        return;
    }

    uint8_t scale_sum = pwm_scale & 0xFF;
    float irun_set = cs_to_amps(IRUN);
    float i_est    = (float)scale_sum / 255.0f * irun_set;

    printf("  TMC status | PWM_SCALE: %3d/255 | est. current: %.3fA | StallGuard: %d | speed: %.0f steps/s\n",
           scale_sum, i_est, sg, mot.current_speed);
}

// Homing
bool motion_home(bool use_stallguard, float homing_speed, int32_t rail_length_steps) {
    mot.homing = true;
    mot.rail_calibrated = false;
    motion_endstop_safety_enable(true);

    // Configure StallGuard for homing if requested
    if (use_stallguard) {
        tmc_write(TMC_REG_SGTHRS, mot.stallguard_threshold);
        tmc_write(TMC_REG_TCOOLTHRS, 0xFFFFF);
    }

    float saved_speed = mot.target_speed;
    mot.target_speed = homing_speed;
    motion_enable(true);

    uint32_t step_interval = (uint32_t)(1000000.0f / homing_speed);

    // Guard: if switch is already triggered before we move, back off first
    if (endstop_debounced(ENDSTOP_PIN, 3, 100)) {
        gpio_put(DIR_PIN, AWAY_DIR);
        sleep_us(10);
        for (int i = 0; i < 200; i++) {
            gpio_put(STEP_PIN, 1); sleep_us(2); gpio_put(STEP_PIN, 0);
            sleep_us(step_interval - 2);
            mot.position++;
        }
        sleep_ms(50);
    }

    // Move toward MIN endstop
    bool hit_min = false;
    uint32_t timeout = 2000000;
    gpio_put(DIR_PIN, HOME_DIR);
    mot.direction = (HOME_DIR == 1);
    sleep_us(10);

    while (!hit_min && timeout-- > 0) {
        gpio_put(STEP_PIN, 1); sleep_us(2); gpio_put(STEP_PIN, 0);
        sleep_us(step_interval - 2);
        if (endstop_debounced(ENDSTOP_PIN, 2, 50)) { hit_min = true; break; }
        if (use_stallguard && tmc_check_stall())     { hit_min = true; break; }
    }

    if (!hit_min) {
        mot.homing = false;
        motion_endstop_safety_enable(false);
        return false;
    }

    mot.position = 0;
    sleep_ms(50);

    // Back off from MIN
    gpio_put(DIR_PIN, AWAY_DIR);
    mot.direction = (AWAY_DIR == 1);
    sleep_us(10);
    for (int i = 0; i < 300; i++) {
        gpio_put(STEP_PIN, 1); sleep_us(2); gpio_put(STEP_PIN, 0);
        sleep_us(step_interval - 2);
        mot.position++;
        if (gpio_get(ENDSTOP_PIN)) break; // released
    }
    for (int i = 0; i < 50; i++) { // clearance
        gpio_put(STEP_PIN, 1); sleep_us(2); gpio_put(STEP_PIN, 0);
        sleep_us(step_interval - 2);
        mot.position++;
    }
    sleep_ms(50);

    // rail_min is the usable minimum
    mot.rail_min = mot.position;

    // Find MAX endstop
    bool hit_max = false;
    timeout = 2000000;
    gpio_put(DIR_PIN, AWAY_DIR);
    mot.direction = (AWAY_DIR == 1);
    sleep_us(10);

    while (!hit_max && timeout-- > 0) {
        gpio_put(STEP_PIN, 1); sleep_us(2); gpio_put(STEP_PIN, 0);
        sleep_us(step_interval - 2);
        mot.position++;

        if ((mot.position % 500) == 0) {
            printf("  Homing MAX... pos=%ld endstop=%d\n",
                mot.position,
                gpio_get(ENDSTOP_PIN));
        }

        if (endstop_debounced(ENDSTOP_PIN, 3, 50)) {
            printf("  MAX endstop detected at pos=%ld\n", mot.position);
            hit_max = true;
            break;
        }

        if (use_stallguard && tmc_check_stall()) {
            printf("  MAX StallGuard detected at pos=%ld\n", mot.position);
            hit_max = true;
            break;
        }
    }

    if (!hit_max) {
        mot.homing = false;
        motion_endstop_safety_enable(false);
        return false;
    }

    // Back off from MAX
    gpio_put(DIR_PIN, HOME_DIR);
    mot.direction = (HOME_DIR == 1);
    sleep_us(10);
    for (int i = 0; i < 300; i++) {
        gpio_put(STEP_PIN, 1); sleep_us(2); gpio_put(STEP_PIN, 0);
        sleep_us(step_interval - 2);
        mot.position--;
        if (gpio_get(ENDSTOP_PIN)) break; // released
    }
    for (int i = 0; i < 50; i++) { // clearance
        gpio_put(STEP_PIN, 1); sleep_us(2); gpio_put(STEP_PIN, 0);
        sleep_us(step_interval - 2);
        mot.position--;
    }
    sleep_ms(50);

    // rail_max is the usable maximum
    mot.rail_max = mot.position;

    // rail_length_steps hint is ignored: always use measured values
    (void)rail_length_steps;

    mot.target_speed    = saved_speed;
    mot.homing          = false;
    mot.rail_calibrated = true;
    motion_endstop_safety_enable(false);

    return true;
}

// Status Queries
int32_t motion_get_position(void) {
    return mot.position;
}

float motion_get_position_mm(float steps_per_mm) {
    return (float)mot.position / steps_per_mm;
}

bool motion_is_moving(void) {
    return mot.moving;
}

bool motion_is_calibrated(void) {
    return mot.rail_calibrated;
}

float motion_get_rail_percent(void) {
    if (!mot.rail_calibrated || mot.rail_max == mot.rail_min) return 0.0f;
    float p = (float)(mot.position - mot.rail_min) /
              (float)(mot.rail_max  - mot.rail_min);
    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;
    return p;
}

int32_t mot_get_rail_min(void) { return mot.rail_min; }
int32_t mot_get_rail_max(void) { return mot.rail_max; }

void tmc_uart_scan_ioin(void) {
    // Test: release TX and just listen
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(UART_TX_PIN, GPIO_IN);
    gpio_pull_up(UART_TX_PIN);
    
    sleep_ms(50);
    
    printf("IDLE RX: ");
    int count = 0;
    while (uart_is_readable(UART_ID)) {
        printf(" %02X", uart_getc(UART_ID));
        count++;
    }
    printf("  (%d bytes)\n", count);
    
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    sleep_ms(10);
    for (uint8_t addr = 0; addr < 4; addr++) {
        uart_drain_rx();

        uint8_t req[4];
        req[0] = 0x05;
        req[1] = addr;
        req[2] = TMC_REG_IOIN & 0x7F;
        req[3] = tmc_crc(req, 3);

        printf("ADDR %d TX:", addr);
        for (int i = 0; i < 4; i++) printf(" %02X", req[i]);
        printf("\n");

        uart_write_blocking(UART_ID, req, 4);
        uart_tx_wait_blocking(UART_ID);

        // Release TX
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);
        gpio_set_dir(UART_TX_PIN, GPIO_IN);
        gpio_disable_pulls(UART_TX_PIN);
        busy_wait_us_32(100);

        sleep_ms(50);

        printf("ADDR %d RX:", addr);
        int count = 0;
        while (uart_is_readable(UART_ID)) {
            printf(" %02X", uart_getc(UART_ID));
            count++;
        }
        printf("  (%d bytes)\n", count);

        // Restore TX
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    }
}