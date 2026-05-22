#pragma once
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─── Pin Configuration ────────────────────────────────────────────────────────
// MR2004B custom Pico 2 W PCB pinout
#define STEP_PIN        11   // TMC2209 STEP
#define DIR_PIN         10   // TMC2209 DIR
#define EN_PIN          28   // TMC2209 EN, active LOW
#define ENDSTOP_PIN     17   // Shared limit-switch input, active LOW with internal pull-up

#define RGB_R_PIN       21
#define RGB_G_PIN       20
#define RGB_B_PIN       19
#define BUZZER_PIN      18

// Change these to 0 if your PCB uses active-LOW/common-anode LED or active-LOW buzzer drive.
#define RGB_ACTIVE_HIGH     1
#define BUZZER_ACTIVE_HIGH  1

// MPU6050 I2C pins on the custom PCB.
// The motor-only test code does not require the MPU to be connected.
#define MPU_I2C_PORT    i2c1
#define MPU_SDA_PIN     2
#define MPU_SCL_PIN     3
#define MPU_I2C_BAUD    400000

// Direction that drives toward the MIN/home endstop.
// If homing goes the wrong way, flip this: 0 → 1 or 1 → 0
#define HOME_DIR        0    // 0 or 1
#define AWAY_DIR        (1 - HOME_DIR)

#define UART_ID         uart1
#define BAUD_RATE       115200
#define UART_TX_PIN     8    // Pico UART1 TX, connected to TMC2209 single-wire UART net
#define UART_RX_PIN     9

// ─── Physical Machine Constants ───────────────────────────────────────────────
#define MOTOR_STEPS_PER_REV  200
#define MICROSTEPS           16

// Keep these for reference only.
// Your real measured calibration overrides the theoretical calculation.
#define PULLEY_TEETH         80
#define BELT_PITCH_MM        2.0f
#define MM_PER_REV           (PULLEY_TEETH * BELT_PITCH_MM)

// ─── Measured machine calibration ───────────────────────────────────────────
// From homing:
// usable travel = 3085 - 61 = 3024 steps
// physical travel = 344.35 mm
// STEPS_PER_MM = 3024 / 344.35 = 8.78
#define RAIL_LENGTH_MM       344.35f
#define STEPS_PER_MM         8.78f

#define RAIL_STEPS           ((int32_t)(RAIL_LENGTH_MM * STEPS_PER_MM))
#define RAIL_CENTER_STEPS    (RAIL_STEPS / 2)

// ─── Current Settings (NEMA 17 — TMC2209 max 2A RMS) ─────────────────────────
// Formula: I_rms = (register_val / 31) * 2.828A
// IRUN 28 = 2.56A peak / 1.81A RMS — fine for short duty-cycle saves with 24V
// If motor gets uncomfortably hot after sustained use, drop to 24
#define IHOLD       8    // Quieter/stronger hold for testing; reduce later if motor gets hot
#define IRUN        20   // Conservative run current for UART-verified setup; raise later only if needed
#define IHOLDDELAY  4    // ~0.5s before dropping to IHOLD after move ends

// ─── Goalkeeper Motion Profiles ───────────────────────────────────────────────
// At 20 steps/mm:
//   20000 steps/sec = 1000 mm/sec  (safe baseline)
//   28000 steps/sec = 1400 mm/sec  (good target)
//   34000 steps/sec = 1700 mm/sec  (likely ceiling for NEMA 17 on 24V)
//   48000+           = step loss territory on most NEMA 17s
//
// TUNING LADDER — raise SAVE_SPEED by 2000 at a time and test:
//   Does it skip? → lower by 2000 and raise SAVE_ACCEL instead.
//   Skipping on accel only? → lower SAVE_ACCEL, keep speed.
//   Skipping at cruise? → lower SAVE_SPEED.


#define SAVE_SPEED          8000.0f   // Faster baseline: ~911 mm/s at 8.78 steps/mm
#define SAVE_ACCEL          40000.0f  // Increase/decrease after testing for skipped steps
#define SAVE_DECEL          40000.0f

#define RETURN_SPEED        5000.0f
#define RETURN_ACCEL        30000.0f

#define HOMING_SPEED        400.0f

// ─── TMC2209 Register Map ────────────────────────────────────────────────────
#define TMC_REG_GCONF       0x00
#define TMC_REG_GSTAT       0x01
#define TMC_REG_IFCNT       0x02   // UART successful-write counter
#define TMC_REG_IOIN        0x06
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_TPOWERDOWN  0x11
#define TMC_REG_TSTEP       0x12
#define TMC_REG_TPWMTHRS    0x13
#define TMC_REG_TCOOLTHRS   0x14
#define TMC_REG_VACTUAL     0x22
#define TMC_REG_SGTHRS      0x40   // StallGuard threshold
#define TMC_REG_SG_RESULT   0x41
#define TMC_REG_COOLCONF    0x42
#define TMC_REG_CHOPCONF    0x6C
#define TMC_REG_PWMCONF     0x70
#define TMC_REG_PWM_SCALE   0x71
#define TMC_REG_MSCNT       0x6A


// ─── Motion Parameters ───────────────────────────────────────────────────────
typedef struct {
    // Speed (steps/sec)
    float max_speed;
    float current_speed;
    float target_speed;
    float manual_velocity_command;  // signed steps/sec; + = toward rail_max, - = toward rail_min
    bool  manual_velocity_mode;     // true while manual velocity control is active

    // Acceleration / S-curve
    float acceleration;       // steps/sec²
    float deceleration;       // steps/sec²
    float jerk;               // steps/sec³ — rate of accel change (S-curve smoothing)
                              // set to 0 to disable S-curve (pure trapezoidal)
    float current_accel;      // live acceleration value (varies during S-curve)

    // Position (steps, absolute)
    int32_t position;
    int32_t target_position;
    int32_t rail_min;
    int32_t rail_max;
    bool    rail_calibrated;

    // Step timing
    uint32_t step_interval_us;
    uint64_t last_step_time;

    // PID controller (position mode)
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_integral;
    float pid_prev_error;
    bool  pid_enabled;

    // State flags
    bool  moving;
    bool  direction;
    bool  enabled;
    bool  homing;

    // StallGuard
    uint8_t stallguard_threshold;
    bool    stall_detected;

    // Step loss tracking
    // motion_update() compares expected vs actual SG_RESULT trend.
    // If stall_callback is set, it fires when a stall is detected while moving.
    void (*stall_callback)(void);  // set via motion_set_stall_callback()
    uint32_t stall_debounce;       // consecutive stall reads before firing callback
} MotionState;

// ─── Public API ──────────────────────────────────────────────────────────────

// Initialisation
void tmc_init(void);

// Board outputs
void rgb_set(bool r, bool g, bool b);
void rgb_off(void);
void buzzer_set(bool on);
void buzzer_beep(uint32_t duration_ms);
bool limit_switch_pressed(void);
void tmc_configure(uint8_t ihold, uint8_t irun, uint8_t iholddelay,
                   uint8_t microsteps, bool stealthchop);
void motion_timer_start(void);  // start hardware-timer step generation (call after homing)

// Motion control
void motion_set_speed(float steps_per_sec);
void motion_set_max_speed(float steps_per_sec);
float motion_get_max_speed(void);
void motion_set_acceleration(float accel, float decel);
void motion_set_jerk(float jerk);           // enable S-curve; 0 = trapezoidal
void motion_move_to(int32_t target_steps);
void motion_move_to_mm(float mm);           // convenience: move to mm position
void motion_move_relative(int32_t delta_steps);

// Manual velocity mode is intended for joystick / Xbox control.
// velocity_steps_per_sec > 0 moves toward rail_max.
// velocity_steps_per_sec < 0 moves toward rail_min.
// velocity_steps_per_sec == 0 decelerates/stops and exits manual mode.
void motion_manual_velocity(float velocity_steps_per_sec);
void motion_manual_stop(void);
bool motion_is_manual_velocity_mode(void);

void motion_stop(void);
void motion_stop_immediate(void);
void motion_enable(bool en);
void motion_endstop_safety_enable(bool en); // normally false after homing; enable only for slow/manual safety moves

// Goalkeeper helpers
void motion_save(float target_mm);          // max-speed move to mm position inside calibrated rail
void motion_save_left(void);             // max-speed move directly to calibrated rail_min
void motion_save_right(void);            // max-speed move directly to calibrated rail_max
void motion_return_center(void);            // smooth return to rail center


// Homing
bool motion_home(bool use_stallguard, float homing_speed, int32_t rail_length_steps);

// PID
void motion_pid_configure(float kp, float ki, float kd);
void motion_pid_enable(bool en);
float motion_pid_compute(int32_t setpoint, int32_t measurement, float dt);

// Update — call as fast as possible in main loop (or from timer ISR)
void motion_update(void);

// Status
int32_t motion_get_position(void);
float   motion_get_position_mm(float steps_per_mm);
bool    motion_is_moving(void);
bool    motion_is_calibrated(void);
float   motion_get_rail_percent(void);
int32_t mot_get_rail_min(void);
int32_t mot_get_rail_max(void);

// TMC2209 UART
void     tmc_write(uint8_t reg, uint32_t value);
uint32_t tmc_read(uint8_t reg);
uint16_t tmc_get_stallguard(void);
bool     tmc_check_stall(void);
void     motion_set_stall_callback(void (*cb)(void)); // called on stall detect while moving

// Current control
void  tmc_set_current(float irun_amps, float ihold_amps);  // set current in amps (max 2.0A RMS)
float tmc_get_current_scale(void);    // returns PWM_SCALE_SUM: 0–255, proportional to actual load
void  tmc_print_status(void);         // prints current scale, stallguard, and computed amps to stdout

// Utilities
uint8_t  tmc_crc(uint8_t *data, uint8_t length);
uint32_t tmc_microstep_bits(uint16_t microsteps); // convert 1/2/4/../256 to MRES bits

void tmc_uart_scan_ioin(void);
void tmc_print_config_status(void);

#ifdef __cplusplus
}
#endif