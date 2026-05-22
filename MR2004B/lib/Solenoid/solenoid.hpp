#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SOLENOID_PIN
#define SOLENOID_PIN 22
#endif

// Safety limit
#ifndef SOLENOID_MAX_PULSE_MS
#define SOLENOID_MAX_PULSE_MS 500u
#endif

typedef void (*solenoid_service_callback_t)(void);

void solenoid_init(void);

void solenoid_trigger(uint32_t pulse_ms);
void solenoid_update(void);
bool solenoid_is_active(void);
void solenoid_force_off(void);

void solenoid_fire_blocking(uint32_t pulse_ms, solenoid_service_callback_t service_callback);

void kick(uint32_t pulse_ms);

#ifdef __cplusplus
}
#endif
