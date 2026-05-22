#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Keep the solenoid pin here now that the TMC2209 library no longer owns it.
#ifndef SOLENOID_PIN
#define SOLENOID_PIN 22
#endif

// Safety limit. Prevents accidentally holding the coil ON for too long.
#ifndef SOLENOID_MAX_PULSE_MS
#define SOLENOID_MAX_PULSE_MS 500u
#endif

typedef void (*solenoid_service_callback_t)(void);

void solenoid_init(void);

// Non-blocking API. Recommended for the Bluetooth/controller main loop.
void solenoid_trigger(uint32_t pulse_ms);
void solenoid_update(void);
bool solenoid_is_active(void);
void solenoid_force_off(void);

// Blocking API that behaves like the old TMC kick() function.
// If service_callback is not NULL, it is called repeatedly while the pulse is active.
// For your current robot, pass motion_update so stepping keeps running during the kick.
void solenoid_fire_blocking(uint32_t pulse_ms, solenoid_service_callback_t service_callback);

// Backward-compatible alias for the old name, but now inside the Solenoid library.
// You can remove this later if you prefer only solenoid_* names.
void kick(uint32_t pulse_ms);

#ifdef __cplusplus
}
#endif
