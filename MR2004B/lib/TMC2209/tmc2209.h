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

// Pin Configuration (On UM-1)
#define STEP_PIN        11   // TMC2209 STEP
#define DIR_PIN         10   // TMC2209 DIR
#define EN_PIN          28   // TMC2209 EN, active LOW
#define ENDSTOP_PIN     17   // Shared limit-switch input

#define RGB_R_PIN       21
#define RGB_G_PIN       20
#define RGB_B_PIN       19
#define BUZZER_PIN      18

#define RGB_ACTIVE_HIGH     1
#define BUZZER_ACTIVE_HIGH  1

// MPU6050 I2C pins on the custom PCB.
#define MPU_I2C_PORT    i2c1
#define MPU_SDA_PIN     2
#define MPU_SCL_PIN     3
#define MPU_I2C_BAUD    400000

#define HOME_DIR        0    // 0 or 1
#define AWAY_DIR        (1 - HOME_DIR)

#define UART_ID         uart1
#define BAUD_RATE       115200
#define UART_TX_PIN     8  
#define UART_RX_PIN     9

#define MOTOR_STEPS_PER_REV  200
#define MICROSTEPS           8

#define PULLEY_TEETH         80
#define BELT_PITCH_MM        2.0f
#define MM_PER_REV           (PULLEY_TEETH * BELT_PITCH_MM)

#define RAIL_LENGTH_MM       344.35f
#define STEPS_PER_MM         8.78f

#define RAIL_STEPS           ((int32_t)(RAIL_LENGTH_MM * STEPS_PER_MM))
#define RAIL_CENTER_STEPS    (RAIL_STEPS / 2)

#define IHOLD       8  
#define IRUN        24
#define IHOLDDELAY  4   



#define SAVE_SPEED          7000.0f  
#define SAVE_ACCEL          55000.0f   // Save acceleration in steps/sec^2
#define SAVE_DECEL          65000.0f   // Save deceleration in steps/sec^2
#define SAVE_JERK          180000.0f   // Save jerk in steps/sec^3;

#define RETURN_SPEED         9000.0f
#define RETURN_ACCEL        30000.0f
#define RETURN_DECEL        35000.0f
#define RETURN_JERK        140000.0f

#define MANUAL_MAX_SPEED    14000.0f
#define MANUAL_ACCEL       110000.0f
#define MANUAL_DECEL       130000.0f
#define MANUAL_JERK         0.0f

#define HOMING_SPEED          400.0f

// TMC2209 Register Map
#define TMC_REG_GCONF       0x00
#define TMC_REG_GSTAT       0x01
#define TMC_REG_IFCNT       0x02 // UART successful-write counter
#define TMC_REG_IOIN        0x06
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_TPOWERDOWN  0x11
#define TMC_REG_TSTEP       0x12
#define TMC_REG_TPWMTHRS    0x13
#define TMC_REG_TCOOLTHRS   0x14
#define TMC_REG_VACTUAL     0x22
#define TMC_REG_SGTHRS      0x40 // StallGuard threshold
#define TMC_REG_SG_RESULT   0x41
#define TMC_REG_COOLCONF    0x42
#define TMC_REG_CHOPCONF    0x6C
#define TMC_REG_PWMCONF     0x70
#define TMC_REG_PWM_SCALE   0x71
#define TMC_REG_MSCNT       0x6A
#define TMC_REG_DRV_STATUS  0x6F


// Motion Parameters
typedef struct {
    
    float max_speed;             
    float current_speed;
    float target_speed;
    float manual_velocity_command;
    bool  manual_velocity_mode;  

    // Acceleration / S-curve
    float acceleration;       // steps/sec²
    float deceleration;       // steps/sec²
    float jerk;               // steps/sec³ 
    float current_accel;      // live acceleration value

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

    void (*stall_callback)(void);  
    uint32_t stall_debounce;      
} MotionState;

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
void motion_timer_start(void);  // start hardware-timer step generation

// Motion control
void motion_set_speed(float steps_per_sec);
void motion_set_max_speed(float steps_per_sec);
float motion_get_max_speed(void);
void motion_set_acceleration(float accel, float decel);
void motion_set_jerk(float jerk);           // enable S-curve; 0 = trapezoidal
float motion_get_jerk(void);
void motion_move_to(int32_t target_steps);
void motion_move_to_mm(float mm);           // convenience: move to mm position
void motion_move_relative(int32_t delta_steps);

void motion_manual_velocity(float velocity_steps_per_sec);
void motion_manual_stop(void);
bool motion_is_manual_velocity_mode(void);

void motion_stop(void);
void motion_stop_immediate(void);
void motion_enable(bool en);
void motion_endstop_safety_enable(bool en); // normally false after homing;

// Goalkeeper helpers
void motion_save(float target_mm);
void motion_save_left(void);    
void motion_save_right(void);     
void motion_return_center(void);   

// Homing
bool motion_home(bool use_stallguard, float homing_speed, int32_t rail_length_steps);

// PID
void motion_pid_configure(float kp, float ki, float kd);
void motion_pid_enable(bool en);
float motion_pid_compute(int32_t setpoint, int32_t measurement, float dt);

// Update
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
void     motion_set_stall_callback(void (*cb)(void)); 

// Current control
void  tmc_set_current(float irun_amps, float ihold_amps); 
float tmc_get_current_scale(void);  
void  tmc_print_status(void);         

// Utilities
uint8_t  tmc_crc(uint8_t *data, uint8_t length);
uint32_t tmc_microstep_bits(uint16_t microsteps); 

void tmc_uart_scan_ioin(void);
void tmc_print_config_status(void);

#ifdef __cplusplus
}
#endif