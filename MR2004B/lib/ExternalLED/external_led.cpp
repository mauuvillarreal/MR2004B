#include "external_led.hpp"

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

namespace external_led {
namespace {
constexpr uint16_t PWM_WRAP = 4095;           // 12-bit brightness resolution
constexpr uint32_t ENVELOPE_UPDATE_MS = 10;   // only updates duty cycle at 100 Hz
constexpr uint16_t BREATH_STEPS = 50;         // 0.50 s up + 0.50 s down (1.0 s total)

int red_gpio = 26;
int green_gpio = 27;
bool red_active_high = true;
bool green_active_high = true;
bool red_on = false;
bool green_on = false;
bool breathing = false;
uint green_slice = 0;
uint green_channel = 0;
uint16_t breath_phase = 0;
uint32_t last_update_ms = 0;

bool valid(int pin) {
    return pin >= 0 && pin < NUM_BANK0_GPIOS;
}

void writeRed(bool on) {
    if (!valid(red_gpio)) return;
    gpio_put(static_cast<uint>(red_gpio), red_active_high ? on : !on);
}

void setGreenDuty(uint16_t level) {
    if (!valid(green_gpio)) return;
    if (level > PWM_WRAP) level = PWM_WRAP;
    if (!green_active_high) level = PWM_WRAP - level;
    pwm_set_chan_level(green_slice, green_channel, level);
}

uint16_t smoothBreathLevel(uint16_t phase) {
    // Convert the up/down phase to 0..4095.
    uint32_t x;
    if (phase < BREATH_STEPS) {
        x = (static_cast<uint32_t>(phase) * PWM_WRAP) / BREATH_STEPS;
    } else {
        x = (static_cast<uint32_t>((2 * BREATH_STEPS) - phase) * PWM_WRAP) / BREATH_STEPS;
    }

    // Integer smoothstep: 3x^2 - 2x^3. It eases both ends and avoids visible steps.
    const uint32_t x2 = (x * x) / PWM_WRAP;
    const uint32_t x3 = (x2 * x) / PWM_WRAP;
    return static_cast<uint16_t>((3u * x2) - (2u * x3));
}
}

void init(int red_pin,
          int green_pin,
          bool red_is_active_high,
          bool green_is_active_high) {
    red_gpio = red_pin;
    green_gpio = green_pin;
    red_active_high = red_is_active_high;
    green_active_high = green_is_active_high;

    if (valid(red_gpio)) {
        gpio_init(static_cast<uint>(red_gpio));
        gpio_set_dir(static_cast<uint>(red_gpio), GPIO_OUT);
        writeRed(false);
    }

    if (valid(green_gpio)) {
        gpio_set_function(static_cast<uint>(green_gpio), GPIO_FUNC_PWM);
        green_slice = pwm_gpio_to_slice_num(static_cast<uint>(green_gpio));
        green_channel = pwm_gpio_to_channel(static_cast<uint>(green_gpio));

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_wrap(&cfg, PWM_WRAP);
        // 150 MHz / 16 / 4096 ≈ 2.29 kHz on Pico 2
        pwm_config_set_clkdiv(&cfg, 16.0f);
        pwm_init(green_slice, &cfg, true);
        setGreenDuty(0);
    }
}

void setRed(bool on) {
    red_on = on;
    writeRed(red_on);
}

void setGreen(bool on) {
    green_on = on;
    if (!breathing) setGreenDuty(green_on ? PWM_WRAP : 0);
}

void setGreenBreathing(bool enabled) {
    if (breathing == enabled) return;

    breathing = enabled;
    breath_phase = 0;
    last_update_ms = to_ms_since_boot(get_absolute_time());

    if (breathing) {
        setGreenDuty(0);
    } else {
        setGreenDuty(green_on ? PWM_WRAP : 0);
    }
}

void update() {
    if (!breathing || !valid(green_gpio)) return;

    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if (static_cast<uint32_t>(now - last_update_ms) < ENVELOPE_UPDATE_MS) return;
    last_update_ms = now;

    breath_phase = static_cast<uint16_t>((breath_phase + 1u) % (2u * BREATH_STEPS));
    setGreenDuty(smoothBreathLevel(breath_phase));
}
}
