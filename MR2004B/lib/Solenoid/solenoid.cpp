#include "solenoid.hpp"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"

static volatile bool solenoid_active = false;
static volatile uint32_t solenoid_off_time_ms = 0;

static uint32_t clamp_pulse(uint32_t pulse_ms) {
    if (pulse_ms > SOLENOID_MAX_PULSE_MS) {
        return SOLENOID_MAX_PULSE_MS;
    }
    return pulse_ms;
}

void solenoid_init(void) {
    gpio_init(SOLENOID_PIN);
    gpio_set_dir(SOLENOID_PIN, GPIO_OUT);
    gpio_put(SOLENOID_PIN, 0);

    solenoid_active = false;
    solenoid_off_time_ms = 0;
}

void solenoid_trigger(uint32_t pulse_ms) {
    pulse_ms = clamp_pulse(pulse_ms);

    if (pulse_ms == 0) {
        solenoid_force_off();
        return;
    }

    gpio_put(SOLENOID_PIN, 1);
    solenoid_active = true;
    solenoid_off_time_ms = to_ms_since_boot(get_absolute_time()) + pulse_ms;
}

void solenoid_update(void) {
    if (!solenoid_active) {
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Signed subtraction handles wraparound safely.
    if ((int32_t)(now - solenoid_off_time_ms) >= 0) {
        solenoid_force_off();
    }
}

bool solenoid_is_active(void) {
    return solenoid_active;
}

void solenoid_force_off(void) {
    gpio_put(SOLENOID_PIN, 0);
    solenoid_active = false;
    solenoid_off_time_ms = 0;
}

void solenoid_fire_blocking(uint32_t pulse_ms, solenoid_service_callback_t service_callback) {
    pulse_ms = clamp_pulse(pulse_ms);

    if (pulse_ms == 0) {
        solenoid_force_off();
        return;
    }

    gpio_put(SOLENOID_PIN, 1);
    solenoid_active = true;

    uint32_t start = to_ms_since_boot(get_absolute_time());

    while ((to_ms_since_boot(get_absolute_time()) - start) < pulse_ms) {
        if (service_callback) {
            service_callback();
        } else {
            tight_loop_contents();
        }
    }

    solenoid_force_off();
}

void kick(uint32_t pulse_ms) {
    solenoid_fire_blocking(pulse_ms, NULL);
}
